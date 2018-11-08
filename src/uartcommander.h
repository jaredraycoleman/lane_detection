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

typedef struct _tUARTCommand
{
    int16_t maxTime;
    int16_t speed;
    int16_t orientation;
    int16_t distance;
    uint8_t dir;
} UARTCommand;


typedef struct _LDMap
{
   uint8_t id;
   uint8_t leaderId;
   int8_t position_x,position_y,position_z; // decimenter 
   int8_t speed_x,  speed_y, speed_z;   // cm per second
   int8_t acceleration_x,acceleration_y,acceleration_z; // cm per second square
   int16_t orientation;
   uint16_t members;
   int16_t distance;
   uint32_t timestamp;
} LDMap;



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
      
      void sendCommand(UARTCommand uartCommand);
      bool isOpen() const;
      void execute();
      void register_callback(std::function<void(const LDMap&)>);
      
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
      uint8_t buffer[sizeof(LDMap)+1];
      uint8_t ch;
      uint8_t lenght;
      uint8_t iByte;
      std::vector<std::function<void(LDMap&)>> callbacks;
      
      std::thread *serialExecution;
      std::mutex mutex;
};


#endif