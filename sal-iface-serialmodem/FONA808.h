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

#ifndef FONA808_H_
#define FONA808_H_

#include "fwk.h"

//#include "at/ATCommandsInterface.h"
#include "PPPIPInterface.h"
//#include "sms/GSMSMSInterface.h"
//#include "sms/CDMASMSInterface.h"
//#include "ussd/USSDInterface.h"
//#include "LinkMonitor.h"
#include "CellularModem.h"
#include "mbed-drivers/mbed.h"
#include "SerialBuffered.h"

/*class genericAtProcessor : public IATCommandsProcessor
{
public:
  genericAtProcessor();
  const char* getResponse(void);
private:
  virtual int onNewATResponseLine(ATCommandsInterface* pInst, const char* line);
  virtual int onNewEntryPrompt(ATCommandsInterface* pInst);
protected:
  char str[256];
  int i;
};
*/
/** u-blox WCDMA modem (LISA-U200)
 */
class FONA808
{
public:
  /** Create u-blox API instance
   */
  FONA808(PinName Tx, PinName rx, PinName rst);

  //Internet-related functions

  /** Open a 3G internet connection
      @return 0 on success, error code on failure
  */
  int connect(const char* apn = NULL, const char* user = NULL, const char* password = NULL);

  /** Close the internet connection
     @return 0 on success, error code on failure
  */
  int disconnect();


  /** Send a SM
     @param number The receiver's phone number
     @param message The message to send
     @return 0 on success, error code on failure
   */
 // virtual int sendSM(const char* number, const char* message);


  /** Receive a SM
     @param number Pointer to a buffer to store the sender's phone number (must be at least 17 characters-long, including the sapce for the null-terminating char)
     @param message Pointer to a buffer to store the the incoming message
     @param maxLength Maximum message length that can be stored in buffer (including null-terminating character)
     @return 0 on success, error code on failure
   */
 // virtual int getSM(char* number, char* message, size_t maxLength);

  /** Get the number of SMs in the incoming box
     @param pCount pointer to store the number of unprocessed SMs on
     @return 0 on success, error code on failure
   */
 // virtual int getSMCount(size_t* pCount);

  /** Send a USSD command & wait for its result
    @param command The command to send
    @param result Buffer in which to store the result
    @param maxLength Maximum result length that can be stored in buffer (including null-terminating character)
    @return 0 on success, error code on failure
  */
 // int sendUSSD(const char* command, char* result, size_t maxLength);
  
  /** Get link state
    @param pRssi pointer to store the current RSSI in dBm, between -51 dBm and -113 dBm if known; -51 dBm means -51 dBm or more; -113 dBm means -113 dBm or less; 0 if unknown
    @param pRegistrationState pointer to store the current registration state
    @param pBearer pointer to store the current bearer
    @return 0 on success, error code on failure
  */
  //int getLinkState(int* pRssi, LinkMonitor::REGISTRATION_STATE* pRegistrationState, LinkMonitor::BEARER* pBearer);  

 // int getPhoneNumber(char* phoneNumber);  

  /** Get the ATCommandsInterface instance
    @return Pointer to the ATCommandsInterface instance
   */
 // virtual ATCommandsInterface* getATCommandsInterface();
  

  /** Initialise dongle.
   * The following actions are performed:
   * 1) Start AT interface thread
   * 2) Wait for network registration
   */
  bool init();
  
  /** De-initialise dongle.
   * The following actions are performed:
   * 1) Tear down PPP session
   * 2) Set SMS,USSD, and LinkMonitor subsystems to un-initialised
   * 3) Close the AT commands interface
   */
  int cleanup();

  uint8_t getNetworkStatus();


  /** 
   * send an AT command and check the reply
   */
  bool sendCheckReply(const char* command, const char* expected, uint16_t timeout);

private:
  SerialBuffered mSerial;    // Serial to which FONA is connected
 // LinkMonitor m_linkMonitor;   //< Interface to link monitor (RSSI) 
  PPPIPInterface m_ppp;        //< Interface to PPP conection manager (IP assignment etc)
  bool m_ipInit;          //< Has PPIPInterface object (m_ppp) been initialised? true/false
  DigitalOut m_rst;       //rst pin of FONA
  bool sendParseReply(char* command, const char *toreply, uint16_t *v, char divider, uint8_t index, uint16_t timeout);
 // bool m_linkMonitorInit; //< Has LinkMonitor object (m_linkMonitor) been initialised? true/false
};
#endif
