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

FONA808::FONA808(PinName Tx, PinName Rx, PinName rst):m_ipInit(false),mSerial(3000,Tx,Rx),m_ppp(&mSerial),m_rst(rst) {

}

int FONA808::connect(const char* apn, const char* user, const char* password){
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
  
  printf("Birra\n");
  
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
         printf("APN set to %s", apn);
    }
   
  }

  //Connect
  printf("Connecting PPP");

  int ret = m_ppp.connect(); //TODO impostare callback per connessione
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
  printf("Ciao FONA808!\n");
  m_rst = 1;         //perform FONA reboot
  wait_ms(100);
  m_rst=0;
  wait_ms(100);
  m_rst = 1;
  
  wait_ms(10000);   // wait for reboot
  
  printf("Cleaning buffer\n");
  mSerial.cleanBuffer();
  printf("Buffer clean\n");
  
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
		printf("Not registered to network yet\n");
		wait_ms(500);
		continue;
	 }
	 return true;
 }
  printf("Max num tries!\n");

  return false;
  
}

bool FONA808::enableGPS(bool enable){
  if(m_ppp.isPPPLinkOpen()){
	mSerial.setPppPause(true); 
	wait_ms(1000);
	mSerial.printf("+++");
	wait_ms(1000);
  }

  uint16_t state;
  if (! sendParseReply("AT+CGPSPWR?","+CGPSPWR: ", &state,',',0,500)){
     	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.setPppPause(false);
	}
	 return false;
     }
  

  if (enable && !state) {
     if (! sendCheckReply("AT+CGPSPWR=1", "OK",500)){
	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.setPppPause(false);	
	}
	return false;
     }
  } else if (!enable && state) {
    if (! sendCheckReply("AT+CGPSPWR=0", "OK",500)){
	if(m_ppp.isPPPLinkOpen()){
		sendCheckReply("ATO","CONNECT",500);
		mSerial.setPppPause(false);
	}
	return false;
     }
  }

  if(m_ppp.isPPPLinkOpen()){
	sendCheckReply("ATO","CONNECT",500);
	mSerial.setPppPause(false);
  }
  return true;
}

uint8_t FONA808::getNetworkStatus(void) {
  uint16_t status;

  if (! sendParseReply("AT+CREG?", "+CREG: ", &status, ',', 1, 500)) return 0;
  printf("Status = %d",status);
  return status;
}

bool FONA808::sendParseReply(char* command, const char *toreply,
          uint16_t *v, char divider, uint8_t index, uint16_t timeout) { 
  char reply[32];
  mSerial.setTimeout(timeout);
  printf("Sending %s\r\n",command);
  //__disable_irq();
  mSerial.printf("%s\r\n",command);
  //__enable_irq();
  mSerial.readline((uint8_t*)reply,32); 
  printf("Got %s", reply);
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

int FONA808::cleanup(){
  return 0;
}

bool FONA808::sendCheckReply(const char* command, const char* reply, uint16_t timeout){
   
  // printf("SendCheckReply\n");
   char replybuf[48];
   //printf("SetTimeout\n");
   mSerial.setTimeout(timeout);
   printf("Sending %s\r\n",command);
   //__disable_irq();
   mSerial.printf("%s\r\n",command);
   //__enable_irq();
   //printf("la bomba\n");
   mSerial.readline((uint8_t*)replybuf,48);  
   printf("Got %s", replybuf);
   printf("baboomba\n");
   return strncmp(replybuf,reply,strlen(reply)) == 0;
}
