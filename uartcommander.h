#ifndef _UARTCOMMANDER_H_
#define _UARTCOMMANDER_H_

#include <iostream>
#include <queue>
#include <thread>
#include <mutex>

#include <vector>
#include <boost/asio.hpp>



using namespace std;
using namespace boost;


struct _tUARTCommand
{
    int8_t speed;
    int8_t wheelOrientation;
    uint8_t maxTime;
    uint8_t orientation;
};

struct _tUARTData
{
    uint8_t id;
    uint8_t speed;
    uint8_t position_x,position_y;
    uint8_t acceleration_x,acceleration_y;
    uint8_t orientation;
    uint8_t timestamp;
};

typedef struct _tUARTCommand UARTCommand;
typedef struct _tUARTData UARTData;

enum {UartWaitForFirstStart, UartWaitForSecondStart,
      UartWaitForLenght, UartWaitForData, UartWaitForCheckSum};


class SerialCommunication
{
    public:
      SerialCommunication(const std::string& devname, unsigned int baud_rate,
 			  asio::serial_port_base::parity opt_parity= asio::serial_port_base::parity(asio::serial_port_base::parity::none),
			  asio::serial_port_base::character_size opt_csize= asio::serial_port_base::character_size(8),
				  asio::serial_port_base::flow_control opt_flow=
				  asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none),
			  asio::serial_port_base::stop_bits opt_stop=asio::serial_port_base::stop_bits(
				  asio::serial_port_base::stop_bits::one));
      
      ~SerialCommunication();
      
      void sendCommand(UARTCommand *uartCommand);
      bool isOpen() const;
      void execute();
      
      void run();
      void join();

private:
    void open(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity=
            asio::serial_port_base::parity(
                asio::serial_port_base::parity::none),
        asio::serial_port_base::character_size opt_csize=
            asio::serial_port_base::character_size(8),
        asio::serial_port_base::flow_control opt_flow=
            asio::serial_port_base::flow_control(
                asio::serial_port_base::flow_control::none),
        asio::serial_port_base::stop_bits opt_stop=
            asio::serial_port_base::stop_bits(
                asio::serial_port_base::stop_bits::one));

      void receiveData(unsigned char c);
      void writeCommand(unsigned char *data, unsigned char size);

    
      std::queue<UARTCommand> queue;
      asio::io_service io;
      asio::serial_port port;
      bool openPort;
      
      uint8_t rxUartState;
      uint8_t buffer[sizeof(UARTData)+1];
      uint8_t ch;
      uint8_t lenght;
      uint8_t iByte;
      
      std::thread *serialExecution;
      std::mutex mutex;
};


#endif