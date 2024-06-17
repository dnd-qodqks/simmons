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
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "result_msgs/msg/mode.hpp"
#include "result_msgs/msg/person.hpp"
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
                "/mode_info", 10, 
                std::bind(&SerialModule::modeCallback, this, std::placeholders::_1));
      
      person_info_sub_ = this->create_subscription<result_msgs::msg::Person>(
                "/person_info", 10, 
                std::bind(&SerialModule::personCallback, this, std::placeholders::_1));
                     
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
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
        return;
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Serial Open!!");
      }
      
      if (tcgetattr(serial_port, &options) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting serial port attributes: %s", strerror(errno));
        close(serial_port);
        return;
      }
      
      bzero(&options, sizeof(options));
      
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
      options.c_iflag = IGNPAR;
      options.c_oflag      = 0;
      options.c_lflag      = 0;
      options.c_cc[VTIME]  = 0;
      options.c_cc[VMIN]   = 0;
      
      tcflush(serial_port, TCIFLUSH);
      
      if (tcsetattr(serial_port, TCSANOW, &options) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting serial port options: %s", strerror(errno));
        close(serial_port);
        return;
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
      //RCLCPP_INFO(this->get_logger(), "Max Elapsed: %f [s]", max_elapsed);
    }
  
  private:
    void modeCallback(const result_msgs::msg::Mode msg) {
      mode_ = msg.mode;
      RCLCPP_DEBUG(this->get_logger(), "Mode: %d", mode_);
      
      if (mode_ != pre_mode_) 
      {
        tcflush(serial_port, TCOFLUSH);
        pre_mode_ = mode_;
      } 
    }
    
    void personCallback(const result_msgs::msg::Person msg) {
      if (std::isnan(msg.length)) {
          person_length_ = 0.0;
      } else {
          person_length_ = msg.length;
      }

      if (std::isnan(msg.degree)) {
          person_degree_ = 0.0;
      } else {
          person_degree_ = msg.degree;
      }
    }

    
    void uart_tx() {
      // TX ---------------------------------
      //auto start = std::chrono::high_resolution_clock::now();
      
      //float person_length = 1.2;
      //float person_degree = 91.3;
      uint8_t length = 8;
      uint8_t tx_data[8];
      
      floatToByteArray(person_length_, tx_data);
      floatToByteArray(person_degree_, &tx_data[4]);
      
      writePacket(length, tx_data);
      RCLCPP_DEBUG(this->get_logger(), "TX) Person Length: %.3f, Person Degree: %.3f", person_length_, person_degree_);
      
      //auto end = std::chrono::high_resolution_clock::now();
      //std::chrono::duration<double> elapsed = end - start;
      
      // if (max_elapsed < elapsed.count()) max_elapsed = elapsed.count();
      
      // RCLCPP_INFO(this->get_logger(), "Elapsed: %.6f", elapsed.count());
    }
    
    void uart_rx() {
      // RX ---------------------------------
      uint8_t rx_data[12] = { 0 };
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
      
      float max = 40000.0, min = 0.0;
      if (FX > max) FX = max;
      if (FY > max) FY = max;
      if (FZ > max) FZ = max;
      if (FX < min) FX = min;
      if (FY < min) FY = min;
      if (FZ < min) FZ = min;

      if (pre_FX != FX || pre_FY != FY || pre_FZ != FZ)
      {
      	RCLCPP_INFO(this->get_logger(), "RX) FX: %.3f, FY: %.3f, FZ: %.3f", FX, FY, FZ);
      }
      
      result_msgs::msg::Force msg;
      msg.x = FX - 20000;
      msg.y = FY - 20000;
      msg.z = FZ - 20000;
      force_info_pub_->publish(msg);
    
      pre_FX = FX;
      pre_FY = FY;
      pre_FZ = FZ;
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
      
      std::memcpy(&value, &tmp, sizeof(float));
      
      return value;
    }

    
    void writePacket(uint8_t length, uint8_t *data) {
      uint8_t txpacket[13] = { 0, };

      if (txpacket == NULL || data == NULL)
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
    }
    
    void txPacket(uint8_t *txpacket) {
      if (txpacket == NULL)
        return;
    
      uint8_t checksum = 0;
      //uint8_t total_packet_length = txpacket[3] + 5; // 5: HEADER0 HEADER1 FLAG LENGTH CHECKSUM
      uint8_t total_packet_length = 13;
      
      txpacket[0] = 0xFF;
      txpacket[1] = 0xFF;
      txpacket[2] = mode_;

      for (int i = 0; i < total_packet_length; i++)
        checksum = addByte(checksum, txpacket[i]);
      checksum = ~checksum;
      
      txpacket[total_packet_length-1] = checksum;

      for (int i = 0; i < 13; i++)
        RCLCPP_DEBUG(this->get_logger(), "txpacket[%d]: %u", i, txpacket[i]);

      uint32_t intr_times = 0;
      uint32_t write_timeout_us_ = 20;
      
      while (total_packet_length > 0)
      {
        ssize_t ret = write(serial_port, txpacket, total_packet_length);
        if (ret == -1)
        {
          if (errno == EINTR || errno == EAGAIN)
          {
            intr_times++;
            if (intr_times > write_timeout_us_)
            {
              errno = EBUSY;
              break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            continue;
          }
          
          break;
        }
        
        intr_times = 0;
        total_packet_length -= ret;
        txpacket 	    += ret;
      }

      if (total_packet_length > 0)
      {
        RCLCPP_DEBUG(this->get_logger(), "TX Packet Fail!! total_packet_length: %u", total_packet_length);
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "TX Packet Success!!");
      }
    }
    
    int readPacket(uint8_t *data, uint8_t total_length) {
      uint8_t rxpacket[RXPACKET_MAX_LEN] = { 0 };
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
            checksum = addByte(checksum, rxpacket[i]);
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
    
    uint8_t addByte(uint8_t a, uint8_t b)
    {
      while (b != 0)
      {
      	uint8_t carry = a & b; // AND
      	
      	a = a ^ b; // XOR
      	
      	b = carry << 1; // Shift
      }
      
      return a;
    }
  
  float person_length_ = 0.0;
  float person_degree_ = 0.0;  
  double max_elapsed = 0.0;
  float pre_FX = 0.0;
  float pre_FY = 0.0;
  float pre_FZ = 0.0;
  int mode_ = 0;
  int pre_mode_ = 0;
  int serial_port;
  struct pollfd fds[1];
  rclcpp::Publisher<result_msgs::msg::Force>::SharedPtr force_info_pub_;
  rclcpp::Subscription<result_msgs::msg::Person>::SharedPtr person_info_sub_;
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

