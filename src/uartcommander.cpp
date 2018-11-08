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

void SerialCommunication::register_callback(std::function<void(const LDMap&)> callback)
{
    callbacks.push_back(callback);
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

void SerialCommunication::sendCommand(UARTCommand uartCommand)
{
    mutex.lock();
     queue.push(uartCommand);
     mutex.unlock();
}

void SerialCommunication::receiveData(unsigned char c)
{
	if (isprint(c) || c == '\n') 
    {
	  // printf("%c", c);
    }
        switch (rxUartState)
        {
            case UartWaitForFirstStart:
                if (c == UART_FIRST_BYTE)
                {
                    
                    rxUartState = UartWaitForSecondStart;
         		    // printf("UART_FIRST_BYTE\n");
                }
                break;
            case UartWaitForSecondStart:
                if (c == UART_SECOND_BYTE)
                {
                    // printf("UART_SECOND_BYTE\n");
                    rxUartState = UartWaitForLenght;
                }
                else
                {
                    rxUartState = UartWaitForFirstStart;
        		    // printf("UART_FIRST_BYTE\n");
                }
                break;
            case UartWaitForLenght:
                if (sizeof(LDMap) == c)
                {
                    ch = 0;
                    lenght = c;
                    iByte = 0;
                    rxUartState = UartWaitForData;
		            // printf("uartData %d\n", lenght);
                }
                else
                {
                    rxUartState = UartWaitForFirstStart;
        		    // printf("UART_FIRST_BYTE\n");
                }
                break;
            case UartWaitForData:
                if (iByte < lenght)
                {
                    // printf("data %d %d\n", iByte, c);
                    ch += c;
                    buffer[iByte++] = c;
                }
                if (iByte == lenght)
                {
                    rxUartState = UartWaitForCheckSum;
		            // printf("UartWaitForCheckSum\n");
                }
                break;
            case UartWaitForCheckSum:
                // accept if the calculated ch is equal to c
	            // printf("Checksum %d %d\n", ch, c);
                if (ch == c)
                {
                    LDMap ldmap;
                    memcpy(&ldmap, buffer, sizeof(LDMap));

                    for (const auto &callback : callbacks)
                    {
                        callback(ldmap);
                    }
                    // printf("\033[2J");
                    // printf("\033[H");
                    // printf("%d Pos (%d,%d,%d), \nSpeed (%d,%d,%d), \nAcc (%d,%d,%d)\nheading %d\n Members %d, distance %d, timestamp %d \n", 
                    // ldmap.id, ldmap.position_x, ldmap.position_y, ldmap.position_z,
                    // ldmap.speed_x, ldmap.speed_y, ldmap.speed_z, ldmap.acceleration_x, ldmap.acceleration_y,
                    // ldmap.acceleration_z, ldmap.orientation, ldmap.members, ldmap.distance, ldmap.timestamp);
                    
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
      unsigned char bufffer[sizeof(LDMap)+5];
      memset(bufffer, 0, sizeof(LDMap)+5);
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
	  std::size_t size = boost::asio::read(port, boost::asio::buffer(bufffer, sizeof(LDMap)+5));
// 	  	  std::size_t size = boost::asio::read(port, boost::asio::buffer(bufffer, 8));

//  	printf("Size %d\n", (int)size);
	 
	  for (i=0; i<size; i++)
	  {
		
//  		printf("Byte %d\n", bufffer[i]);
	      receiveData(bufffer[i]);
	  }
	      
	  memset(bufffer, 0, sizeof(LDMap)+5);
      }
}
      

