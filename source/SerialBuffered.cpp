
#include "mbed-drivers/mbed.h"
#include "SerialBuffered.h"
 
//extern Serial loggerSerial;
 
SerialBuffered::SerialBuffered( size_t bufferSize, PinName tx, PinName rx ) : RawSerial(  tx,  rx ), loggerSerial(USBTX,USBRX) 
{
    m_buffSize = 0;
    m_contentStart = 0;
    m_contentEnd = 0;
    m_timeout = 1.0;
   
    
    attach( this, &SerialBuffered::handleInterrupt, RxIrq );
    //attach( this, &SerialBuffered::dummy, TxIrq );
   
    m_buff = (uint8_t *) malloc( bufferSize );
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


int SerialBuffered::readable()
{
    return m_contentStart != m_contentEnd ;
}

void SerialBuffered::handleInterrupt()
{
   __disable_irq();
    static DigitalOut led(LED2);
    led=!led;
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
    led=!led;
   __enable_irq();
}



