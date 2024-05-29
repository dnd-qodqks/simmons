#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
 
#define MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

class SerialModule : public rclcpp::Node {
  public:
    SerialModule() : Node("serial_node") {
      force_info_pub_ = this->create_publisher<std_msgs::msg::String>(
                "/force_info", 10);
                
      timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&SerialModule::publish_message, this));
          
      struct termios options;
      
      const char* device = "/dev/ttyAMA0";
      
      serial_port = open(device, O_RDWR | O_NOCTTY);
      if (serial_port == -1) {
        RCLCPP_INFO(this->get_logger(), "Serial Open Error");
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Serial Open!!");
      }
      
      if (tcgetattr(serial_port, &options) != 0) {
        RCLCPP_INFO(this->get_logger(), "termios Error");
      }
      
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_cflag &= ~CRTSCTS;
      options.c_cflag |= CREAD | CLOCAL;  
      options.c_lflag &= ~ICANON;
      options.c_lflag &= ~ECHO;
      options.c_lflag &= ~ECHOE;
      options.c_lflag &= ~ECHONL;
      options.c_lflag &= ~ISIG;
      options.c_iflag &= ~(IXON | IXOFF | IXANY);
      options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);  
      options.c_oflag &= ~OPOST;
      options.c_oflag &= ~ONLCR;  
      options.c_cc[VTIME] = 10;
      options.c_cc[VMIN] = 0;
      
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      
      if (tcsetattr(serial_port, TCSANOW, &options)!= 0) {
        RCLCPP_INFO(this->get_logger(), "UART Option Error");
      }
      else {
        RCLCPP_INFO(this->get_logger(), "UART Option Setting!!");
      }

      is_using_ = false;
    }
    
    ~SerialModule() {
      if (serial_port != -1) {
        close(serial_port);
      }
      RCLCPP_INFO(this->get_logger(), "Close Serial Module");
    }
  
  private:
    void publish_message() {
      float data1 = 123456.789;
      float data2 = 6.9;
      float data3 = 34.78;
      int length = 12;
      uint8_t tx_data[12];
      
      floatToByteArray(data1, tx_data);
      floatToByteArray(data2, &tx_data[4]);
      floatToByteArray(data3, &tx_data[8]);
      
      writePacket(length, tx_data);
      //RCLCPP_INFO(this->get_logger(), "tx_data: '%u', '%u', '%u', '%u'", tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
      
      uint8_t rx_data[12] = {0};
      readPacket(length, rx_data);
      //RCLCPP_INFO(this->get_logger(), "rx_data: '%u', '%u', '%u', '%u'", rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
      
      data1 = byteArrayToFloat(rx_data);
      data2 = byteArrayToFloat(&rx_data[4]);
      data3 = byteArrayToFloat(&rx_data[8]);
      
      RCLCPP_INFO(this->get_logger(), "data: '%f', '%f', '%f'", data1, data2, data3);
      
      /* // Used int
      uint8_t tx_data[4] = { LOBYTE(LOWORD(data)), HIBYTE(LOWORD(data)), LOBYTE(HIWORD(data)), HIBYTE(HIWORD(data)) };
      writePacket(length, tx_data);
      RCLCPP_INFO(this->get_logger(), "tx_data: '%x', '%x', '%x', '%x'", tx_data[0], tx_data[1], tx_data[2], tx_data[3]);

      uint8_t rx_data[4] = {0};
      readPacket(length, rx_data);
      
      data = MAKEDWORD(MAKEWORD(rx_data[0], rx_data[1]), MAKEWORD(rx_data[2], rx_data[3]));
      RCLCPP_INFO(this->get_logger(), "rx_data: '%u', '%u', '%u', '%u'", rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
      RCLCPP_INFO(this->get_logger(), "data: '%f'", data);
      */
    }

    void serial_test() {
      if (serial_port == -1) {
        RCLCPP_INFO(this->get_logger(), "serial_port == -1");
        return;
      }
    
      RCLCPP_INFO(this->get_logger(), "Start Pub!!");
      const char* message = "Hello, UART!";
      write(serial_port, message, strlen(message));
      RCLCPP_INFO(this->get_logger(), "Write!! : '%s'", message);
      
      char buffer[1024];
      memset(&buffer, '\0', sizeof(buffer));
      
      int num_bytes = read(serial_port, buffer, sizeof(buffer));
      if (num_bytes < 0) {
        RCLCPP_ERROR(this->get_logger(), "buffer Error");
      }
      
      RCLCPP_INFO(this->get_logger(), "buffer: '%s'", buffer);
    }

    void floatToByteArray(float value, uint8_t* buffer) {
      // float 변수의 메모리 주소를 unsigned char 포인터로 변환
      unsigned char* bytes = reinterpret_cast<unsigned char*>(&value);
    
      // 각 바이트를 uint8_t 배열에 저장
      for (size_t i = 0; i < sizeof(value); ++i) {
        buffer[i] = bytes[i];
      }
    }

    float byteArrayToFloat(const uint8_t* array) {
      unsigned long tmp;
      float result;
      
      tmp = ((array[3] & 0xFF) << 24);
      tmp += ((array[2] & 0xFF) << 16);
      tmp += ((array[1] & 0xFF) << 8);
      tmp += array[0] & 0xFF;
      
      result = *((float*) & tmp);
      
      return result;
    }

    void writePacket(uint8_t length, uint8_t *data) {
      uint8_t *txpacket = (uint8_t *)malloc(length+5);

      if (txpacket == NULL)
        return;

      txpacket[3] = length + 5;

      for (uint8_t s = 0; s < length; s++)
      {
        txpacket[4+s] = data[s];
      }

      txPacket(txpacket);
      is_using_ = false;

      free(txpacket);
    }

    void readPacket(uint8_t length, uint8_t *data) {
      uint8_t *rxpacket = (uint8_t *)malloc(20);
      int result = 0;

      if (rxpacket == NULL)
        return;

      do {
        result = rxPacket(rxpacket);
      } while (result == 1);

      for (uint8_t s = 0; s < length; s++)
      {
        data[s] = rxpacket[4 + s];
      }

      free(rxpacket);
    }

    void txPacket(uint8_t *txpacket) {
      uint8_t checksum = 0;
      uint8_t total_packet_length = txpacket[3] + 4; // 4: HEADER0 HEADER1 FLAG LENGTH
      uint8_t written_packet_length  = 0;

      if (is_using_)
        return;
      is_using_ = true;

      txpacket[0] = 0xFF;
      txpacket[1] = 0xFF;

      for (uint16_t idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
        checksum += txpacket[idx];
      txpacket[total_packet_length - 1] = ~checksum;

      tcflush(serial_port, TCIFLUSH);

      written_packet_length = write(serial_port, txpacket, total_packet_length);

      if (total_packet_length != written_packet_length)
      {
        is_using_ = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "TX Packet Success!!");
    }

    int rxPacket(uint8_t *rxpacket) {
      auto start = std::chrono::high_resolution_clock::now();
      double timeout = 0.003; 
      uint8_t checksum       = 0;
      uint8_t rx_length      = 0;
      uint8_t wait_length    = 5;    // minimum length (HEADER0 HEADER1 FLAG LENGTH CHKSUM)
      int result = 0;

      while (true) {
        rx_length += read(serial_port, &rxpacket[rx_length], wait_length - rx_length);

        if (rx_length >= wait_length) {
          uint8_t idx = 0;

          // find packet header
          for (idx = 0; idx < (rx_length - 1); idx++)
          {
            if (rxpacket[idx] == 0xFF && rxpacket[idx+1] == 0xFF)
              break;
          }
	  
	  
          if (idx == 0)   // found at the beginning of the packet
          {
            if (rxpacket[3] > 20)  // unavailable Length 20: RXPACKET_MAX_LEN
            {
                // remove the first byte in the packet
                for (uint16_t s = 0; s < rx_length - 1; s++)
                  rxpacket[s] = rxpacket[1 + s];
                //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
                rx_length -= 1;
                continue;
            }

            // re-calculate the exact length of the rx packet
            if (wait_length != rxpacket[3] + 5)
            {
              wait_length = rxpacket[3] + 5;
              continue;
            }

            if (rx_length < wait_length)
            {
              // check timeout
              auto finish = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double> elapsed = finish - start;
              if (elapsed.count() > timeout)
              {
                if (rx_length == 0)
                {
                  RCLCPP_INFO(this->get_logger(), "TIMEOUT!!");
                  result = -1;
                }
                else
                {
                  RCLCPP_INFO(this->get_logger(), "CORRUPT!!");
                  result = -1;
                }
                break;
              }
              else
              {
                continue;
              }
            }

            // calculate checksum
            for (uint16_t i = 2; i < wait_length - 1; i++)   // except header, checksum
              checksum += rxpacket[i];
            checksum = ~checksum;

            // verify checksum
            if (rxpacket[wait_length - 1] == checksum)
            {
              RCLCPP_INFO(this->get_logger(), "SUCCESS!!");
              result = 1;
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "CORRUPT!!");
              result = -1;
            }
            break;
          }
        }
        else
        {
          // check timeout
          auto finish = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed = finish - start;
          if (elapsed.count() > timeout)
          {
            if (rx_length == 0)
            {
              RCLCPP_INFO(this->get_logger(), "TIMEOUT!!");
              result = -1;
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "CORRUPT!!");
              result = -1;
            }
            break;
          }
        }
      }

      is_using_ = false;

      return result;
    }
  
  int serial_port;
  bool is_using_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialModule>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

