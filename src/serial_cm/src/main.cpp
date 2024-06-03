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
#include <poll.h>

#include <rclcpp/rclcpp.hpp>
#include "result_msgs/msg/mode.hpp"
#include "result_msgs/msg/force.hpp"

#define LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#define RXPACKET_MAX_LEN 17

class SerialModule : public rclcpp::Node {
  public:
    SerialModule() : Node("serial_node") {
      force_info_pub_ = this->create_publisher<result_msgs::msg::Force>(
                "/force_info", 10);
      
      mode_sub_ = this->create_subscription<result_msgs::msg::Mode>(
                "mode_info", 10, 
                std::bind(&SerialModule::modeCallback, this, std::placeholders::_1));
      
      tx_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(10),
                    std::bind(&SerialModule::uart_tx, this));

      rx_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(10),
                    std::bind(&SerialModule::uart_rx, this));
          
      struct termios options;
      memset (&options, 0, sizeof options);
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
      
      bzero(&options, sizeof(options));
      
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
      options.c_iflag = IGNPAR;
      options.c_oflag      = 0;
      options.c_lflag      = 0;
      options.c_cc[VTIME]  = 0;
      options.c_cc[VMIN]   = 0;
      
      tcflush(serial_port, TCIFLUSH);
      
      if (tcsetattr(serial_port, TCSANOW, &options)!= 0) {
        RCLCPP_INFO(this->get_logger(), "UART Option Error");
      }
      else {
        RCLCPP_INFO(this->get_logger(), "UART Option Setting!!");
      }
      
      fds[0].fd = serial_port;
      fds[0].events = POLLRDNORM;
    }
    
    ~SerialModule() {
      if (serial_port != -1) {
        close(serial_port);
      }
      RCLCPP_INFO(this->get_logger(), "Close Serial Module");
    }
  
  private:
    void modeCallback(const result_msgs::msg::Mode msg) {
      mode_ = msg.mode;
      RCLCPP_INFO(this->get_logger(), "Mode: %d", mode_);
    }

    void uart_tx() {
      // TX ---------------------------------
      float person_length = 1.2;
      float person_degree = 91.3;
      uint8_t length = 8;
      uint8_t tx_data[8];
      
      floatToByteArray(person_length, tx_data);
      floatToByteArray(person_degree, &tx_data[4]);
      
      writePacket(length, tx_data);
      RCLCPP_INFO(this->get_logger(), "writePacket!!");
    }
    
    void uart_rx() {
      // RX ---------------------------------
      uint8_t *rx_data = (uint8_t *)malloc(12);
      uint8_t total_length = RXPACKET_MAX_LEN;
      uint8_t result = 0;
      
      while (1)
      {
        result = readPacket(rx_data, total_length);
        if (result == 1)
          break;
      }
      
      for (int i = 0; i < 12; i++)
      {
        RCLCPP_DEBUG(this->get_logger(), "rx_data[%d]: %u", i, rx_data[i]);
      }
      
      float FX = byteArrayToFloat(rx_data);
      float FY = byteArrayToFloat(&rx_data[4]);
      float FZ = byteArrayToFloat(&rx_data[8]);
      
      float max = 10000.0, min = -max;
      if (FX > 10000.0) FX = max;
      if (FY > 10000.0) FY = max;
      if (FZ > 10000.0) FZ = max;
      if (FX < -10000.0) FX = min;
      if (FY < -10000.0) FY = min;
      if (FZ < -10000.0) FZ = min;

      RCLCPP_INFO(this->get_logger(), "FX: %.3f, FY: %.3f, FZ: %.3f", FX, FY, FZ);
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
      uint32_t tmp;
      float value;
      
      tmp = ((array[0] & 0xFF) << 24);
      tmp |= ((array[1] & 0xFF) << 16);
      tmp |= ((array[2] & 0xFF) << 8);
      tmp |= array[3] & 0xFF;
      
      value = *((float*) & tmp);
      
      return value;
    }

    void writePacket(uint8_t length, uint8_t *data) {
      uint8_t *txpacket = (uint8_t *)malloc(length+5);

      if (txpacket == NULL)
        return;

      if (mode_ == 1) {
        txpacket[3] = length;

        for (uint8_t s = 0; s < length; s++)
        {
          txpacket[4+s] = data[s];
        }
      }
      else if (mode_ == 2) {
        txpacket[3] = 0;
      }
      
      txPacket(txpacket);

      free(txpacket);
    }
    
    void txPacket(uint8_t *txpacket) {
      uint8_t checksum = 0;
      uint8_t total_packet_length = txpacket[3] + 5; // 4: HEADER0 HEADER1 FLAG LENGTH CHECKSUM
      uint8_t written_packet_length = 0;
      
      txpacket[0] = 0xFF;
      txpacket[1] = 0xFF;
      txpacket[2] = mode_ - 1;

      if (mode_ == 1)
      {
        checksum = ~(uint8_t)(txpacket[0]
                            + txpacket[1]
                            + txpacket[2]
                            + txpacket[3]
                            + txpacket[4]
                            + txpacket[5]
                            + txpacket[6]
                            + txpacket[7]
                            + txpacket[8]
                            + txpacket[9]
                            + txpacket[10]
                            + txpacket[11]
                            + txpacket[12]);
      }
      else if (mode_ == 2)
      {
        checksum = ~(uint8_t)(txpacket[0]
                            + txpacket[1]
                            + txpacket[2]
                            + txpacket[3]
                            + txpacket[4]);
      }
      
      // for (int i = 0; i < total_packet_length; i++)
      //   checksum += txpacket[i];
      // checksum = ~checksum;

      txpacket[total_packet_length-1] = checksum;

      tcflush(serial_port, TCIFLUSH);

      for (int i = 0; i < total_packet_length; i++)
        RCLCPP_DEBUG(this->get_logger(), "txpacket[%d]: %u", i, txpacket[i]);

      written_packet_length = write(serial_port, txpacket, total_packet_length);

      RCLCPP_DEBUG(this->get_logger(), "TX Packet Success!! written_packet_length: %u", written_packet_length);
    }
    
    int readPacket(uint8_t *data, uint8_t total_length) {
      uint8_t *rxpacket = (uint8_t *)malloc(RXPACKET_MAX_LEN);
      uint8_t rx_length = 0;
      uint8_t checksum  = 0;
      
      tcflush(serial_port, TCIFLUSH);
      int ret = poll(fds, 1, 1000);
      RCLCPP_DEBUG(this->get_logger(), "ret: %d", ret);
      
      if (ret > 0)
      {
        if (fds[0].revents & POLLRDNORM)
        {
          rx_length = read(serial_port, &rxpacket[rx_length], RXPACKET_MAX_LEN);
          RCLCPP_DEBUG(this->get_logger(), "rx_length: %u", rx_length);
                  
          for (int i = 0; i < total_length; i++)
            RCLCPP_DEBUG(this->get_logger(), "rxpacket[%d]: %u", i, rxpacket[i]);
                  
          if (rxpacket[0] != 0xFF || rxpacket[1] != 0xFF)
          {
            RCLCPP_WARN(this->get_logger(), "Not Found Header");
            return -1;
          }
          
          for (int i = 0; i < total_length-1; i++)
            checksum += rxpacket[i];
          checksum = ~checksum;
          
          if (rxpacket[total_length-1] != checksum)
          {
            RCLCPP_DEBUG(this->get_logger(), "rxpacket[total_length-1]: %u", rxpacket[total_length-1]);
            RCLCPP_WARN(this->get_logger(), "Not Same Checksum: %u", checksum);
            return -2;
          }

	  for (int i = 0; i < rxpacket[3]; i++)
	    data[i] = rxpacket[i+4];
        }
      }
      
      return 1;
    }

  int mode_ = 0;
  int serial_port;
  struct pollfd fds[1];
  rclcpp::Publisher<result_msgs::msg::Force>::SharedPtr force_info_pub_;
  rclcpp::Subscription<result_msgs::msg::Mode>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialModule>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}