#include "uartcommander.h"
#include "logger.h"
//#include <boost/thread.hpp>
#include <iostream>

using namespace std;
using namespace boost;

#define UART_FIRST_BYTE  'C'
#define UART_SECOND_BYTE  'O'


SerialCommunication::SerialCommunication(const std::string& devname,
        unsigned int baud_rate,
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
     Logger::getLogger().log("serial", "constructed", Levels::TRACE, {});
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
    Logger::getLogger().log("serial", "opened", Levels::DEBUG, {});
}

bool SerialCommunication::isOpen() const
{
    return openPort;
}

void SerialCommunication::sendCommand(UARTCommand *uartCommand)
{
    Logger::getLogger().log("serial", "sending", Levels::DEBUG, {"comm", "out"});
    mutex.lock();
     queue.push(*uartCommand);
     mutex.unlock();
     Logger::getLogger().log("serial", "sent", Levels::DEBUG, {"comm", "out"});
}

void SerialCommunication::receiveData(unsigned char c)
{
        //printf("Byte %d %d %d\n", c, rxUartState, UartWaitForFirstStart);
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
                if (sizeof(LDMap) == c)
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
                  Logger::getLogger().log("serial", "received", Levels::DEBUG, {"comm", "in"});

                    LDMap ldmap;
                    memcpy(&ldmap, buffer, sizeof(LDMap));
                    callback(ldmap);
            		    // printf("\033[2J");
            		    // printf("\033[H");
                    std::ostringstream oss;
                    oss << "pos: (" << (int)ldmap.position_x  << ", " <<  (int)ldmap.position_y << ", " <<  (int)ldmap.position_z << ")";
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "position"});
                    oss.str(std::string());
                    oss << "speed: (" <<  (int)ldmap.speed_x  << ", " <<  (int)ldmap.speed_y << ", " <<  (int)ldmap.speed_z << ")";
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "speed"});
                    oss.str(std::string());
                    oss << "acceleration: (" <<  (int)ldmap.acceleration_x  << ", " <<  (int)ldmap.acceleration_y << ", " <<  (int)ldmap.acceleration_z << ")";
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "acceleration"});
                    oss.str(std::string());
                    oss << "heading: " <<  (int)ldmap.orientation;
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "heading", "orientation"});
                    oss.str(std::string());
                    oss << "members: " <<  (int)ldmap.members;
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "members"});
                    oss.str(std::string());
                    oss << "distance: " <<  (int)ldmap.distance;
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "distance"});
                    oss.str(std::string());
                    oss << "timestamp: " <<  (int)ldmap.timestamp;
                    Logger::getLogger().log("serial", oss.str(), Levels::DEBUG, {"sensor", "timestamp"});

                }
                rxUartState = UartWaitForFirstStart;
                break;
        }
}


void SerialCommunication::writeCommand(unsigned char *data, unsigned char size)
{

    Logger::getLogger().log("serial", "writing command", Levels::DEBUG, {"comm", "out"});
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

void SerialCommunication::run(std::function<void(LDMap)> callback)
{
     this->callback = callback;
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

      	//printf("Size %d\n", (int)size);

    	  for (i=0; i<size; i++)
    	  {

      		//printf("Byte %d\n", bufffer[i]);
    	      receiveData(bufffer[i]);
    	  }

    	  memset(bufffer, 0, sizeof(LDMap)+5);
      }
}
