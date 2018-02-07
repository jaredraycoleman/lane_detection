#include "uartcommander.h"
//#include <boost/thread.hpp>

using namespace std;
using namespace boost;

#define UART_FIRST_BYTE  'C'
#define UART_SECOND_BYTE  'O'




SerialCommunication::SerialCommunication(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop) : port(io)
{
     ch = 0;
     rxUartState = UartWaitForFirstStart;
     openPort = false;
     iByte = 0;
     serialExecution = NULL;

     open(devname,baud_rate,opt_parity,opt_csize,opt_flow,opt_stop);
}


SerialCommunication::~SerialCommunication()
{
      if(!isOpen()) return;    
	port.close();

}

void SerialCommunication::open(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
{
    if(isOpen()) return;

    
    port.open(devname);
    port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(opt_parity);
    port.set_option(opt_csize);
    port.set_option(opt_flow);
    port.set_option(opt_stop);

    openPort=true; //Port is now open
}

bool SerialCommunication::isOpen() const
{
    return openPort;
}

void SerialCommunication::sendCommand(UARTCommand *uartCommand)
{
    mutex.lock();
     queue.push(*uartCommand);
     mutex.unlock();
}

void SerialCommunication::receiveData(unsigned char c)
{
//         printf("Byte %d %d %d\n", c, rxUartState, UartWaitForFirstStart);
        switch (rxUartState)
        {
            case UartWaitForFirstStart:
                if (c == UART_FIRST_BYTE)
		{
		    
                    rxUartState = UartWaitForSecondStart;
// 		    printf("UART_FIRST_BYTE\n");
		}
                break;
            case UartWaitForSecondStart:
                if (c == UART_SECOND_BYTE)
		{
// 		    printf("UART_SECOND_BYTE\n");
                    rxUartState = UartWaitForLenght;
		}
                else
		{
                    rxUartState = UartWaitForFirstStart;
// 		   printf("UART_FIRST_BYTE\n");
		}
                break;
            case UartWaitForLenght:
                if (sizeof(UARTData) == c)
                {
                    ch = 0;
                    lenght = c;
                    iByte = 0;
                    rxUartState = UartWaitForData;
// 		    printf("uartData %d\n", lenght);
                }
                else
		{
                    rxUartState = UartWaitForFirstStart;
// 		    printf("UART_FIRST_BYTE\n");
		}
                break;
            case UartWaitForData:
                if (iByte < lenght)
                {
//                     printf("data %d %d\n", iByte, c);
                    ch += c;
                    buffer[iByte++] = c;
                }
                if (iByte == lenght)
                {
                    rxUartState = UartWaitForCheckSum;
// 		    printf("UartWaitForCheckSum\n");
                }
                break;
            case UartWaitForCheckSum:
                //  accept if the calculated ch is equal to c
// 	      printf("Checksum %d %d\n", ch, c);
                if (ch == c)
                {
                    UARTData uartData;
                    memcpy(&uartData, buffer, sizeof(UARTData));
// 		    printf("\033[2J");
// 		    printf("\033[H");
                    printf("Receive %d %d %d\n", uartData.timestamp, uartData.speed, uartData.orientation);
                }
                rxUartState = UartWaitForFirstStart;
                break;
        }
}

      
void SerialCommunication::writeCommand(unsigned char *data, unsigned char size)
{
  
    unsigned char buff[3] = {UART_FIRST_BYTE, UART_SECOND_BYTE, size};
    boost::asio::write(port, boost::asio::buffer(buff, 3));
    boost::asio::write(port, boost::asio::buffer(data, size));
    unsigned ch = 0;
    for (int i=0; i<size; i++, data++)
    {
	ch = (ch + *data) % 256;
    }
    buff[0] = ch;
    buff[1] = '\r';
    buff[2] = '\n'; 
    boost::asio::write(port, boost::asio::buffer(&ch, 1));
    boost::asio::write(port, boost::asio::buffer(&buff[0], 1));
    boost::asio::write(port, boost::asio::buffer(&buff[1], 1));

    
//      printf("Sending %d\n", ch);
}

void serialTask(SerialCommunication *serial)
{
     serial->execute();
} 

void SerialCommunication::join()
{
    serialExecution->join();
}

void SerialCommunication::run()
{
     serialExecution = new std::thread(serialTask, this); 
}

void SerialCommunication::execute()
{
      uint8_t i;
      unsigned char bufffer[sizeof(UARTData)+5];
      memset(bufffer, 0, sizeof(UARTData)+5);
      while(true)
      {
	  mutex.lock();
	  if (queue.size() > 0)
	  {
	      UARTCommand command = queue.front();
	      writeCommand((unsigned char *)&command, sizeof(UARTCommand));
	      queue.pop();
	  }
	  mutex.unlock();
	  std::size_t size = boost::asio::read(port, boost::asio::buffer(bufffer, sizeof(UARTData)+5));
	  
	  for (i=0; i<size; i++)
	  {
	      receiveData(bufffer[i]);
	  }
	      
	  memset(bufffer, 0, sizeof(UARTData)+5);
      }
}
      

