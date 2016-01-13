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

FONA808::FONA808(PinName Tx, PinName Rx, PinName rst):m_ipInit(false),mSerial(256,Tx,Rx),m_ppp(&mSerial),m_rst(rst) {

}

int FONA808::connect(const char* apn, const char* user, const char* password){
 //TODO controllare se FONA è stato inizializzato
 if( !m_ipInit )
  {
    m_ipInit = true;
    m_ppp.init();
  }
  m_ppp.setup(user, password, DEFAULT_MSISDN_GSM);
  
  bool in = init();
  if(!in)
  {
    return -1;
  }
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
	 DBG("ATResult: AT return=%d (code %d)", result.result, result.code);
         DBG("APN set to %s", apn);
    }
   
  }

  //Connect
  DBG("Connecting");
  DBG("Connecting PPP");

  int ret = m_ppp.connect(); //TODO impostare callback per connessione
  DBG("Result of connect: Err code=%d", ret);
  return ret;
}

int FONA808::disconnect(){
   return 0;
}

bool FONA808::init(){
  mSerial.baud(4800);
  
  m_rst = 1;         //perform FONA reboot
  wait_ms(10);
  m_rst=0;
  wait_ms(100);
  m_rst = 1;

  wait_ms(7000);   // wait for reboot
  
  mSerial.cleanBuffer();

  sendCheckReply("AT", "OK",100);
  wait_ms(100);
  sendCheckReply("AT", "OK",100);
  wait_ms(100);
  sendCheckReply("AT", "OK",100);
  wait_ms(100);

  // turn off Echo!
  sendCheckReply("ATE0", "OK",100);
  wait_ms(100);

  if (! sendCheckReply("ATE0", "OK",100)) {
    return false;
  }

  while(getNetworkStatus()!=1){
	printf("Not registered to network yet\n");
        wait_ms(500);
 }

 return true;
  
}

uint8_t FONA808::getNetworkStatus(void) {
  uint16_t status;

  if (! sendParseReply("AT+CREG?", "+CREG: ", &status, ',', 1, 500)) return 0;

  return status;
}

bool FONA808::sendParseReply(char* command, const char *toreply,
          uint16_t *v, char divider, uint8_t index, uint16_t timeout) { 
  char reply[32];
  mSerial.setTimeout(timeout/1000);
  mSerial.printf(command);
  mSerial.readline((uint8_t*)reply,32); 
  DBG("Got %s", reply);
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

bool FONA808::sendCheckReply(char* command, const char* reply, uint16_t timeout){
   char replybuf[48];
   mSerial.setTimeout(timeout/1000);
   mSerial.printf(command);
   mSerial.readline((uint8_t*)replybuf,48);  
   DBG("Got %s", replybuf);
   return strncmp(replybuf,reply,strlen(reply)) == 0;
}
