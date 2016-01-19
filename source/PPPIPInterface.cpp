/* PPPIPInterface.cpp */
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

#define __DEBUG__ 1
#ifndef __MODULE__
#define __MODULE__ "PPPIPInterface.cpp"
#endif

#include "fwk.h"
//#include "rtos.h"

#include <cstdio>
using std::sscanf;
using std::sprintf;

#include "PPPIPInterface.h"

#define MSISDN "*99#"

#define CONNECT_CMD_PREFIX "ATD "
#define CONNECT_CMD_SUFFIX "\x0D"
#define EXPECTED_RESP_SUFFIX "\x0D" "\x0A" "CONNECT" "\x0D" "\x0A"
#define EXPECTED_RESP_DATARATE_SUFFIX "\x0D" "\x0A" "CONNECT %d" "\x0D" "\x0A"
#define EXPECTED_RESP_MIN_LEN 20
#define OK_RESP "\x0D" "\x0A" "OK" "\x0D" "\x0A"
#define ESCAPE_SEQ "+++"
#define HANGUP_CMD "ATH" "\x0D"
#define NO_CARRIER_RESP "\x0D" "\x0A" "NO CARRIER" "\x0D" "\x0A"
extern "C" {
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/err.h"
#include "lwip/dns.h"

#include "ppp.h"
}

PPPIPInterface::PPPIPInterface(SerialBuffered* mSerial) : LwIPInterface(), m_pppErrCode(0), m_streamAvail(true), m_pppd(-1)
{
    m_pStream = mSerial;
}



/*virtual*/ PPPIPInterface::~PPPIPInterface()
{
}

/*virtual*/ int PPPIPInterface::init() //Init PPP-specific stuff, create the right bindings, etc
{
  printf("Initializing LwIP");
  LwIPInterface::init(); //Init LwIP, NOT including PPP
  printf("Initializing PPP");
  pppInit();
  printf("Done");
  return OK;
}

int PPPIPInterface::setup(const char* user, const char* pw, const char* msisdn)
{
  printf("Configuring PPP authentication method\n");
  pppSetAuth(PPPAUTHTYPE_ANY, user, pw);
  m_msisdn = msisdn;
  printf("Done\n");
  return OK;
}

/*virtual*/ int PPPIPInterface::connect() //TODO callback connection
{
  int ret;
  char cmd[32];
  int cmdLen;
  char buf[32];
  size_t len;
  printf("Trying to connect with PPP\n");
  
  cleanupLink();
  
  cmdLen = sprintf(cmd, "%s%s%s", CONNECT_CMD_PREFIX, m_msisdn, CONNECT_CMD_SUFFIX);
  printf("Sending %s", cmd);
  //ret = m_pStream->write((uint8_t*)cmd, cmdLen, osWaitForever);
  ret = m_pStream->printf("%s", cmd);
  if( ret < 0 )
  {
    return NET_UNKNOWN;
  }
  printf("Connected\n");
  len = 0;
  size_t readLen;
 /* while(m_pStream->available() && (buf[len]=m_pStream->getc()) != LF && len <EXPECTED_RESP_MIN_LEN){
      len++;
  }
  /*ret = m_pStream->read((uint8_t*)buf + len, &readLen, EXPECTED_RESP_MIN_LEN, 10000);
  
  if( ret != OK )
  {
    return NET_UNKNOWN;
  }
  len += readLen;
  while( (len < EXPECTED_RESP_MIN_LEN) || (buf[len-1] != LF) )
  {
    ret = m_pStream->read((uint8_t*)buf + len, &readLen, 1, 10000);
    if( ret != OK )
    {
      return NET_UNKNOWN;
    }
    len += readLen;
  }
  */
  printf("Trying to read response\n");
  m_pStream->setTimeout(10000);
  len = m_pStream->readBytes((uint8_t*)buf,EXPECTED_RESP_MIN_LEN);
  
  buf[len]=0;
  
  printf("Got %s[len %d]\n", buf, len);
  
  int datarate = 0;
 /* strcpy(&cmd[cmdLen], EXPECTED_RESP_DATARATE_SUFFIX);
  if( (sscanf(buf, cmd, &datarate ) != 1)) 
  {
    strcpy(&cmd[cmdLen], EXPECTED_RESP_SUFFIX);
    if (strcmp(cmd, buf) != 0)
    {
      //Discard buffer
      m_pStream->cleanBuffer();
      return NET_CONN;
    }
  }   */ 
  m_pStream->cleanBuffer();
  printf("Transport link open\n");
  if(datarate != 0)
  {
    printf("Datarate: %d bps\n", datarate);
  }
  //m_linkStatusSphre.wait(0);
  if((m_pppd != -1) && (m_pppErrCode == 0)) //Already connected
  {
    return NET_INVALID;
  }
  //__disable_irq();
  ret = pppOverSerialOpen(this, PPPIPInterface::linkStatusCb, this);
  //__enable_irq();
  printf("PPPoverSerial %d\n",ret);
  if(ret < 0)
  {
    switch(ret)
    {
    case PPPERR_OPEN:
    default:
      return NET_FULL; //All available resources are already used
    }
  }
  mbed::util::FunctionPointer0<void> ptr(this,&PPPIPInterface::pppReadRoutine);
  pppReadHandle = minar::Scheduler::postCallback(ptr.bind()).period(minar::milliseconds(500)).getHandle();
  m_pppd = ret; 
  return 0;
  // TODO: set event for connection / disconnection (callback)

  //PPP descriptor
  //m_linkStatusSphre.wait(); //Block indefinitely; there should be a timeout there
  /*if(m_pppErrCode != PPPERR_NONE)
  {
    m_pppd = -1;
  }
  switch(m_pppErrCode)
  {
  case PPPERR_NONE: //Connected OK
    return OK;
  case PPPERR_CONNECT: //Connection lost
    return NET_INTERRUPTED;
  case PPPERR_AUTHFAIL: //Authentication failed
    return NET_AUTH;
  case PPPERR_PROTOCOL: //Protocol error
    return NET_PROTOCOL;
  default:
    return NET_UNKNOWN;
  }*/
}

void PPPIPInterface::connectionCallback(){
  printf("ConnCallback\n");
  
}

/*virtual*/ int PPPIPInterface::disconnect()
{
  int ret = 0;// m_linkStatusSphre.wait(0); TODO 
  if(ret > 0) //Already disconnected?
  {
    m_pppd = -1; //Discard PPP descriptor
    switch(m_pppErrCode)
      {
      case PPPERR_CONNECT: //Connection terminated
      case PPPERR_AUTHFAIL: //Authentication failed
      case PPPERR_PROTOCOL: //Protocol error
      case PPPERR_USER:
        return OK;
      default:
        return NET_UNKNOWN;
      }
  }
  else
  {
    if(m_pppd == -1)
    {
      return NET_INVALID;
    }
    pppClose(m_pppd);

  }
  /*  do
    {
     // m_linkStatusSphre.wait(); //Block indefinitely; there should be a timeout there
      printf("Received PPP err code %d", m_pppErrCode);
    } while(m_pppErrCode != PPPERR_USER);
    m_pppd = -1; //Discard PPP descriptor
  }
  
  printf("Sending %s", ESCAPE_SEQ);
  
 // ret = m_pStream->write((uint8_t*)ESCAPE_SEQ, strlen(ESCAPE_SEQ), osWaitForever);
  ret = m_pStream->printf(ESCAPE_SEQ);
  if( ret < 0 )
  {
    return NET_UNKNOWN;
  }
  
  cleanupLink();*/
  
  return OK;
}

/*
 * To be scheduled periodically 
 */

void PPPIPInterface::pppReadRoutine(){
  printf("\npppReadRoutine");
  uint8_t buffer[256];
  m_pStream->setTimeout(100);
  int read = m_pStream->readBytes(buffer,256);
  printf("\nRead = %d",read);
  if(read>0){
    pppos_input(m_pppd, buffer,read);
  }
}

void PPPIPInterface::disconnectionCallback(){
  minar::Scheduler::cancelCallback(pppReadHandle);
  printf("Sending %s", ESCAPE_SEQ);
 
 // ret = m_pStream->write((uint8_t*)ESCAPE_SEQ, strlen(ESCAPE_SEQ), osWaitForever);
 int ret = m_pStream->printf(ESCAPE_SEQ);
  if( ret < 0 )
  {
    return;
  }

  m_pppd = -1; //discard ppp descriptor
  
  cleanupLink();
}

int PPPIPInterface::getPPPErrorCode(){
  return m_pppErrCode;
}


int PPPIPInterface::cleanupLink()
{
  int ret;
 /* char buf[32];
  size_t len;
  
  do //Clear buf  TODO DONE
  {
    ret = m_pStream->read((uint8_t*)buf, &len, 32, 100);
    if(ret == OK)
    {
      buf[len] = '\0';
      printf("Got %s", buf);
    }
  } while( (ret == OK) && (len > 0) );*/

  //clear buffer

  m_pStream->cleanBuffer();
  
  printf("Sending %s\n", HANGUP_CMD);
  
  //ret = m_pStream->write((uint8_t*)HANGUP_CMD, strlen(HANGUP_CMD), osWaitForever);
  ret = m_pStream->printf(HANGUP_CMD);
  if( ret < 0 )
  {
    return NET_UNKNOWN;
  }
     
 /* size_t readLen;
  
  //Hangup
  printf("Expect %s", HANGUP_CMD);

  len = 0;
  while( len < strlen(HANGUP_CMD) )
  {
    ret = m_pStream->read((uint8_t*)buf + len, &readLen, strlen(HANGUP_CMD) - len, 100);
    if( ret != OK )
    {
      break;
    }
    len += readLen;
   buf[len]=0;
   printf("Got %s", buf);
    buf[len] = m_pStream->getc();
    len++;
    /////
   
  }
 
  
  buf[len]=0;
  
  printf("Got %s[len %d]", buf, len);
  
  //OK response
  printf("Expect %s", OK_RESP);

  len = 0;
  while( len < strlen(OK_RESP) )
  {
    ret = m_pStream->read((uint8_t*)buf + len, &readLen, strlen(OK_RESP) - len, 100);
    if( ret != OK )
    {
      break;
    }
    len += readLen;
    /////
    buf[len]=0;
    printf("Got %s", buf);
    buf[len] = m_pStream->getc();
    len++;
  }
  
  buf[len]=0;
  
  printf("Got %s[len %d]", buf, len);
  
  //NO CARRIER event
  printf("Expect %s", NO_CARRIER_RESP);

  len = 0;
  while( len < strlen(NO_CARRIER_RESP) )
  {
    ret = m_pStream->read((uint8_t*)buf + len, &readLen, strlen(NO_CARRIER_RESP) - len, 100);
    if( ret != OK )
    {
      break;
    }
    len += readLen;
    /////
    buf[len]=0;
    printf("Got %s", buf);
    buf[len] = m_pStream->getc();
    len++;
  }
  
  buf[len]=0;
  
  printf("Got %s[len %d]", buf, len);
  
  do //Clear buf
  {
    ret = m_pStream->read((uint8_t*)buf, &len, 32, 100);
    if(ret == OK)
    {
      buf[len] = '\0';
      printf("Got %s", buf);
    }
  } while( (ret == OK) && (len > 0) );*/
  
  
  return OK;
}

/*static*/ void PPPIPInterface::linkStatusCb(void *ctx, int errCode, void *arg) //PPP link status
{
  printf("Status callback, error code: %d",errCode);
  PPPIPInterface* pIf = (PPPIPInterface*)ctx;
  struct ppp_addrs* addrs = (struct ppp_addrs*) arg;

  pIf->m_pppErrCode = errCode;
  switch(errCode)
  {
  case PPPERR_NONE:
    WARN("Connected via PPP.");
    printf("Local IP address: %s", inet_ntoa(addrs->our_ipaddr));
    printf("Netmask: %s", inet_ntoa(addrs->netmask));
    printf("Remote IP address: %s", inet_ntoa(addrs->his_ipaddr));
    printf("Primary DNS: %s", inet_ntoa(addrs->dns1));
    printf("Secondary DNS: %s", inet_ntoa(addrs->dns2));
    //Setup DNS
    if (addrs->dns1.addr != 0)
    {
      dns_setserver(0, (struct ip_addr*)&(addrs->dns1));
    }
    if (addrs->dns2.addr != 0)
    {
      dns_setserver(1, (struct ip_addr*)&(addrs->dns1));
    }
        
    pIf->setConnected(true);
    pIf->setIPAddress(inet_ntoa(addrs->our_ipaddr));
    pIf->connectionCallback();
    break;
  case PPPERR_CONNECT: //Connection lost
    WARN("Connection lost/terminated");
    pIf->setConnected(false);
    pIf->disconnectionCallback();
    break;
  case PPPERR_AUTHFAIL: //Authentication failed
    WARN("Authentication failed");
    pIf->setConnected(false);
    pIf->disconnectionCallback();
    break;
  case PPPERR_PROTOCOL: //Protocol error
    WARN("Protocol error");
    pIf->setConnected(false);
    pIf->disconnectionCallback();
    break;
  case PPPERR_USER:
    WARN("Disconnected by user");
    pIf->setConnected(false);
    pIf->disconnectionCallback();
    break;
  default:
    WARN("Unknown error (%d)", errCode);
    pIf->setConnected(false);
    pIf->disconnectionCallback();
    break;
  }

  /*pIf->m_linkStatusSphre.wait(0); //If previous event has not been handled, "delete" it now
  pIf->m_linkStatusSphre.release();*/
  
}


//LwIP PPP implementation
extern "C"
{

/**
 * Writes to the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data to send
 * @param len length (in bytes) of data to send
 * @return number of bytes actually sent
 *
 * @note This function will block until all data can be sent.
 */
u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
  printf("sio_write\n");
  PPPIPInterface* pIf = (PPPIPInterface*)fd;
  int ret;
  if(!pIf->m_streamAvail) //If stream is not available (it is a shared resource) don't go further
  {
    return 0;
  }
  //ret = pIf->m_pStream->write(data, len, osWaitForever); //Blocks until all data is sent or an error happens
  int i = 0;
  while(i < len){
     pIf->m_pStream->putc(data[i]);
     i++;
  }
   printf("Written %d bytes\n",len);
  return len;
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
/*
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  printf("sio_read");
  PPPIPInterface* pIf = (PPPIPInterface*)fd;
  int ret;
  size_t readLen;
  if(!pIf->m_streamAvail) //If stream is not available (it is a shared resource) don't go further
  {
    WARN("EXIT NOT AVAIL");
    return 0;
  }
  ret = pIf->m_pStream->read(data, &readLen, len, osWaitForever); //Blocks until some data is received or an error happens
  if(ret != OK)
  {
    return 0;
  }
  printf("ret");
  return readLen;
}
*/
/**
 * Aborts a blocking sio_read() call.
 *
 * @param fd serial device handle
 */

void sio_read_abort(sio_fd_t fd)
{
 return;
}


}

