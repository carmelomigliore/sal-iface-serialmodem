
#include "mbed-drivers/mbed.h"
#include "SerialBuffered.h"
 
//extern Serial loggerSerial;
 
SerialBuffered::SerialBuffered( size_t bufferSize, PinName tx, PinName rx ) : Serial(  tx,  rx ), loggerSerial(USBTX,USBRX) 
{
    m_buffSize = 0;
    m_contentStart = 0;
    m_contentEnd = 0;
    m_timeout = 1.0;
   
    
    attach( this, &SerialBuffered::handleInterrupt );
    
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

void SerialBuffered::setTimeout( float seconds )
{
    m_timeout = seconds;
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
    m_timer.reset();
    m_timer.start();
    while( m_contentStart == m_contentEnd )
    {
      

        wait_ms( 1 );
        if( m_timeout > 0 &&  m_timer.read() > m_timeout )
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
    
    while( Serial::readable())
    {
        if( m_contentStart == (m_contentEnd +1) % m_buffSize)
        {
           loggerSerial.printf("SerialBuffered - buffer overrun, data lost!\r\n" );
           Serial::getc();

        }
        else
        {
          
            m_buff[ m_contentEnd ++ ] = Serial::getc();
            m_contentEnd = m_contentEnd % m_buffSize;
            
           
           
        }
    }
}

