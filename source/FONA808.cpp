/* Copyright (C) 2012 mbed.org, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#define __DEBUG__ 3
#ifndef __MODULE__
#define __MODULE__ "FONA808.cpp"
#endif

#include "fwk.h"
#include "FONA808.h"
#include <cstring>

FONA808::FONA808(PinName Tx, PinName Rx, PinName rst, mbed::util::FunctionPointer3<void, float, float, float> myGpsReadCallback, mbed::util::FunctionPointer0<void> myGpsNoFixCallback):m_ipInit(false),mSerial(3000,Tx,Rx),m_ppp(&mSerial),m_rst(rst), gpsReadCallback(myGpsReadCallback), gpsNoFixCallback(myGpsNoFixCallback) {
	timestamp[0] = 0;
}

int FONA808::connect(const char* apn, const char* user, const char* password, mbed::util::FunctionPointer0<void> connectionCallback){
bool in = init();
if(!in)
  {
    return -1;
  }
 if( !m_ipInit )
  {
    m_ipInit = true;
    m_ppp.init();
  }
  m_ppp.setup(user, password, DEFAULT_MSISDN_GSM);
  
  DEBUG_PRINT("Birra\n");
  
/*
  if (m_onePort)
  {
     m_smsInit = false; //SMS status reset
     m_ussdInit = false; //USSD status reset
     m_linkMonitorInit = false; //Link monitor status reset
  }*/

  if(apn != NULL)
  {
    char cmd[48];
    int tries = 30;
    sendCheckReply("ATH","OK",500);
    sprintf(cmd, "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
   /* do //Try 30 times because for some reasons it can fail *a lot* with the K3772-Z dongle
    {
      ret = m_at.executeSimple(cmd, &result);
      DBG("Result of command: Err code=%d", ret);
      if(ret)
      {
        Thread::wait(500);
      }
    } while(ret && --tries);*/
    if(!sendCheckReply(cmd,"OK",500)){
	 //printf("ATResult: AT return=%d (code %d)", result.result, result.code);
         DEBUG_PRINT("APN set to %s", apn);
    }
   
  }

  //Connect
  DEBUG_PRINT("Connecting PPP");

  int ret = m_ppp.connect(connectionCallback); //TODO impostare callback per connessione
  //printf("Result of connect: Err code=%d", ret);
  return ret;
}

bool FONA808::isConnected(){
	return m_ppp.isConnected();
}

int FONA808::disconnect(){
   return 0;
}

bool FONA808::init(){
  mSerial.baud(115200);
  DEBUG_PRINT("Ciao FONA808!\n");
  m_rst = 1;         //perform FONA reboot
  wait_ms(200);
  m_rst=0;
  wait_ms(200);
  m_rst = 1;
  
  wait_ms(2000);   // wait for reboot
  
  DEBUG_PRINT("Cleaning buffer\n");
  mSerial.cleanBuffer();
  DEBUG_PRINT("Buffer clean\n");
  mSerial.printf("+++");
  
  wait_ms(1000);
  for(int tries=0; tries < 3; tries++){
	  sendCheckReply("AT", "OK",500);
	  wait_ms(100);
	  sendCheckReply("AT", "OK",500);
	  wait_ms(100);
	  sendCheckReply("AT", "OK",500);
	  wait_ms(100);

	  // turn off Echo!
	  sendCheckReply("ATE0", "OK",5000);
	  wait_ms(100);

	  if (! sendCheckReply("ATE0", "OK",5000)) {
	    continue;
	  }

	  if(getNetworkStatus()!=1){
		DEBUG_PRINT("Not registered to network yet\n");
		wait_ms(500);
		continue;
	 }
	 return true;
 }
  DEBUG_PRINT("Max num tries!\n");

  return false;
  
}

void FONA808::readGPSInfo(){
	DEBUG_PRINT("ReadGPS\n");	
	if(m_ppp.isPPPLinkOpen()){
		mSerial.setPppPause(true);
		mbed::util::FunctionPointer0<void> ptr(this, &FONA808::readGPSInfoFirst);
		minar::Scheduler::postCallback(ptr.bind()).delay(minar::milliseconds(2000));
	}
	else{
		mbed::util::FunctionPointer0<void> ptr2(this, &FONA808::readGPSInfoSecond);
		minar::Scheduler::postCallback(ptr2.bind());
	}
}

void FONA808::readGPSInfoFirst(){
	mSerial.printf("+++");
	mbed::util::FunctionPointer0<void> ptr(this, &FONA808::readGPSInfoSecond);
	minar::Scheduler::postCallback(ptr.bind()).delay(minar::milliseconds(2000));
	DEBUG_PRINT("ReadGPS First\n");
}

void FONA808::readGPSInfoSecond(){
	DEBUG_PRINT("ReadGPS Second\n");
	mSerial.cleanBuffer();
	float lat=0, latitude;
	float lon=0, longitude;
	float elevation=0;
	if(checkGPSFix()){
		mSerial.cleanBuffer();
		sendParseReplyGPS(&lat, &lon, &elevation, this->timestamp, 500);
		float degrees = floor(lat / 100);
	    	double minutes = lat - (100 * degrees);
	    	minutes /= 60;
	    	degrees += minutes;
	    	latitude = degrees;
	    	
	    	degrees = floor(lon / 100);
	    	minutes = lon - (100 * degrees);
	    	minutes /= 60;
	    	degrees += minutes;
		longitude = degrees;
		DEBUG_PRINT("\nRAW Latitude: %d, Longitude=%d\n",(int)lat, (int)lon);
		minar::Scheduler::postCallback(gpsReadCallback.bind(latitude, longitude, elevation));
	}else{
		minar::Scheduler::postCallback(gpsNoFixCallback.bind());
	}
	
	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.cleanBuffer();
		mSerial.setPppPause(false);
 	}

}

bool FONA808::enableGPS(bool enable){
  if(m_ppp.isPPPLinkOpen()){
	mSerial.setPppPause(true); 
	wait_ms(1000);
	mSerial.printf("+++");
	wait_ms(1000);
	mSerial.cleanBuffer();
  }

  uint16_t state;
  if (! sendParseReply("AT+CGPSPWR?","+CGPSPWR: ", &state,',',0,500)){
     	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.cleanBuffer();
		mSerial.setPppPause(false);
	}
	 return false;
     }
  	DEBUG_PRINT("Stato GPS: %d", state);

  if (enable && !state) {
     if (! sendCheckReply("AT+CGPSPWR=1", "OK",500)){
	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.cleanBuffer();
		mSerial.setPppPause(false);	
	}
	return false;
     }
  } else if (!enable && state) {
    if (! sendCheckReply("AT+CGPSPWR=0", "OK",500)){
	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.cleanBuffer();
		mSerial.setPppPause(false);
	}
	return false;
     }
  }

  if(m_ppp.isPPPLinkOpen()){
	sendCheckReply("ATO","CONNECT",500);
	mSerial.cleanBuffer();
	mSerial.setPppPause(false);
  }
  return true;
}

const char* FONA808::getTimestamp() const{
	return timestamp;
}

uint8_t FONA808::getNetworkStatus(void) {
  uint16_t status;
  if (! sendParseReply("AT+CREG?", "+CREG: ", &status, ',', 1, 500)) return 0;
  DEBUG_PRINT("Status = %d",status);
  return status;
}

bool FONA808::sendParseReply(char* command, const char *toreply,
          uint16_t *v, char divider, uint8_t index, uint16_t timeout) { 
  char reply[32];
  mSerial.setTimeout(timeout);
  DEBUG_PRINT("Sending %s\r\n",command);
  //__disable_irq();
  mSerial.printf("%s\r\n",command);
  //__enable_irq();
  mSerial.readline((uint8_t*)reply,32); 
  DEBUG_PRINT("Got %s", reply);
  char *p = strstr(reply, toreply);  // get the pointer to the voltage
  if (p == 0) return false;
  p+=strlen(toreply);
  //Serial.println(p);
  for (uint8_t i=0; i<index;i++) {
    // increment dividers
    p = strchr(p, divider);
    if (!p) return false;
    p++;
    //Serial.println(p);

  }
  *v = atoi(p);
  return true;
}

bool FONA808::checkGPSFix(){
	char replybuf[48];
	const char* command = "AT+CGPSSTATUS?";
	mSerial.setTimeout(500);
	DEBUG_PRINT("Sending %s\r\n",command);
	mSerial.printf("%s\r\n",command);
	mSerial.readline((uint8_t*)replybuf,48);  
	DEBUG_PRINT("Got %s", replybuf);
	mSerial.cleanBuffer();
	const char* fix2d = "+CGPSSTATUS: Location 2D Fix";
	const char* fix3d = "+CGPSSTATUS: Location 3D Fix";
	return (strncmp(replybuf,fix2d,strlen(fix2d)) == 0) || (strncmp(replybuf,fix3d,strlen(fix3d)) == 0);
}

bool FONA808::sendParseReplyGPS(float* lat, float* lon, float* alt, char* utctime, uint16_t timeout) { 
  char reply[96];
  char tmp[30];
  const char* command = "AT+CGPSINF=0";
  const char* toreply = "+CGPSINF: 0,";
  char divider = ',';
  mSerial.setTimeout(timeout);
  DEBUG_PRINT("Sending %s\r\n",command);
  //__disable_irq();
  mSerial.printf("%s\r\n",command);
  //__enable_irq();
  mSerial.readline((uint8_t*)reply,96); 
  DEBUG_PRINT("Got %s", reply);
  char *p = strstr(reply, toreply); 
  if (p == 0) return false;
  
  p+=strlen(toreply);    //p at latitude beginning
  char* end = strchr(p, divider);
  strncpy(tmp, p, end-p);
  tmp[end-p]=0;  //Teminator
  *lat = atof(tmp);
  
  p+= (end-p)+1;       //p at longitude beginning 
  end = strchr(p, divider);
  strncpy(tmp, p, end-p);
  tmp[end-p]=0;
  *lon = atof(tmp);
  
  p+= (end-p)+1;     //p at latitude beginning
  end = strchr(p, divider);
  strncpy(tmp, p, end-p);
  tmp[end-p]=0;
  *alt = atof(tmp);
   
  p+= (end-p)+1;     //p at utc time beginning
  end = strchr(p, divider);
  strncpy(tmp, p, end-p);
  tmp[end-p]=0;
  
   strncpy(utctime,tmp,4); //convert timestamp to ISO format
   utctime[4]='-';
   strncpy(utctime+5,tmp+4,2);
   utctime[7]='-';
   strncpy(utctime+8,tmp+6,2);
   utctime[10]='T';
   strncpy(utctime+11,tmp+8,2);
   utctime[13]=':';
   strncpy(utctime+14,tmp+10,2);
   utctime[16]=':';
   strncpy(utctime+17,tmp+12,2);
   utctime[19]='.';
   utctime[20]='0';
   utctime[21]='0';
   utctime[22]='0';
   utctime[23]='Z';
   utctime[24]='\0';
   
   mSerial.cleanBuffer();
  return true;
}

int FONA808::cleanup(){
  return 0;
}

bool FONA808::sendCheckReply(const char* command, const char* reply, uint16_t timeout){
   
  // printf("SendCheckReply\n");
   char replybuf[48];
   //printf("SetTimeout\n");
   mSerial.setTimeout(timeout);
   DEBUG_PRINT("Sending %s\r\n",command);
   //__disable_irq();
   mSerial.printf("%s\r\n",command);
   //__enable_irq();
   //printf("la bomba\n");
   //DEBUG_PRINT("BUBBA");
   mSerial.readline((uint8_t*)replybuf,48);  
   DEBUG_PRINT("Got %s", replybuf);
   //DEBUG_PRINT("baboomba\n");
   return strncmp(replybuf,reply,strlen(reply)) == 0;
}
