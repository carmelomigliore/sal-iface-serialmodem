
#include "mbed-drivers/mbed.h"
#include "SerialBuffered.h"
 
//extern Serial loggerSerial;
 
extern "C" {
#include "ppp.h"
}

SerialBuffered::SerialBuffered( size_t bufferSize, PinName tx, PinName rx ) : RawSerial(  tx,  rx ), loggerSerial(USBTX,USBRX), isPppOpen(false), m_pppd(-1) 
{
    m_buffSize = 0;
    m_contentStart = 0;
    m_contentEnd = 0;
    m_timeout = 1.0;
   
    
    attach( this, &SerialBuffered::handleInterrupt, RxIrq );
    //attach( this, &SerialBuffered::dummy, TxIrq );
   
    m_buff = (uint8_t *) malloc( bufferSize );
    m_pppbuf = (uint8_t *) malloc(1024);
    if( m_buff == NULL )
    {
        loggerSerial.printf("SerialBuffered - failed to alloc buffer size %d\r\n", (int) bufferSize );
    }
    else
    {
        m_buffSize = bufferSize;
    }
}


SerialBuffered::~SerialBuffered()
{
    if( m_buff )
        free( m_buff );
    if( m_pppbuf )
        free( m_pppbuf );
}

void SerialBuffered::setTimeout( int milliseconds )
{
    m_timeout = milliseconds;
}
    
size_t SerialBuffered::readBytes( uint8_t *bytes, size_t requested )
{
    int i = 0;

    for( ; i < requested; )
    {
        int c = getc();
        if( c < 0 )
            break;
        bytes[i] = c;
        i++;
    }
    
    return i;
        
}

size_t SerialBuffered::readline( uint8_t *bytes, size_t requested )
{
    int i = 0;

    for( ; i < requested; )
    {
        int c = getc();
        if( c < 0)
            break;

        if (c == '\r') continue;
        if (c == 0xA) {
        if (i == 0)   // the first 0x0A is ignored
          continue;
        else
          break;
        }
        bytes[i] = c;
        i++;
    }
    if(i<requested)
      bytes[i] = 0; //terminatore
    return i;
        
}


int SerialBuffered::getc()
{
   //static DigitalOut led(LED2);
    m_timer.reset();
    m_timer.start();
    while( m_contentStart == m_contentEnd )
    {
        wait_ms( 1 );
        if( m_timeout > 0 &&  m_timer.read_ms() > m_timeout )
            return EOF;
    }
    m_timer.stop();
   
    uint8_t result = m_buff[m_contentStart++];
    m_contentStart =  m_contentStart % m_buffSize;
 
   
    return result;
}

void SerialBuffered:: cleanBuffer()
{
    while (readable()){
        getc();
   }

}

void SerialBuffered::sendToPpp(){
  loggerSerial.printf("\npppReadRoutine");
  uint8_t buffer[256];
  setTimeout(100);
  int read = readBytes(buffer,256);
  loggerSerial.printf("\nRead = %d",read);
  if(read>0 && m_pppd != -1){
    pppos_input(m_pppd, buffer,read);
  }
}

void SerialBuffered::setPppOpen(int pppd, bool pppOpen){
        m_pppd = pppd;
	isPppOpen = pppOpen;
}


int SerialBuffered::readable()
{
    return m_contentStart != m_contentEnd ;
}

void SerialBuffered::handleInterrupt()
{
   __disable_irq();
    while( RawSerial::readable())
    {
	if( m_contentStart == (m_contentEnd +1) % m_buffSize)
	{
	   loggerSerial.printf("SerialBuffered - buffer overrun, data lost!\r\n" );
	   RawSerial::getc();
	}
	else
	{
	    m_buff[ m_contentEnd ++ ] = RawSerial::getc();
	    m_contentEnd = m_contentEnd % m_buffSize;
	}
    }
    if(isPppOpen){
          mbed::util::FunctionPointer0<void> ptr(this,&SerialBuffered::sendToPpp);
          minar::Scheduler::postCallback(ptr.bind());
    }
   __enable_irq();
}



