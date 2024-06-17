#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "result_msgs/msg/bbox.hpp"
#include "result_msgs/msg/person.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <cmath>

class ImageProcessingModule : public rclcpp::Node {
    public:
        ImageProcessingModule() : Node("image_processing_node") {
            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/robot/D435/aligned_depth_to_color/image_raw", 10,
                std::bind(&ImageProcessingModule::depthCallback, this,
                            std::placeholders::_1));

	    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/robot/D435/color/image_raw", 10, 
                std::bind(&ImageProcessingModule::imageCallback, this, 
                           std::placeholders::_1));
            
            bbox_sub_ = this->create_subscription<result_msgs::msg::Bbox>(
                 "/bbox", 10, std::bind(&ImageProcessingModule::bboxCallback, this, 
                 std::placeholders::_1));

            person_info_pub_ = this->create_publisher<result_msgs::msg::Person>(
                "/person_info", 10);

            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/yolo_image", 10);
        }
    
    private:
        cv::Mat depth_image;
        uint32_t x1_ = 0;
        uint32_t y1_ = 0;
        uint32_t x2_ = 0;
        uint32_t y2_ = 0;
        result_msgs::msg::Person person_info;

        float get_degree(uint32_t center_x, uint32_t center_y, float depth) {
            double c_x = 329.529296875;
            //double c_y = 240.3614959716797;
            double f_x = 602.3125610351562;
            //double f_y = 602.2564697265625;

            double point_x = (double)depth * ((double)center_x - c_x) / f_x;
            //double point_y = (double)depth * ((double)center_y - c_y) / f_y;

            double angle_in_radian = std::atan2(depth, point_x);
            double angle_in_degree = angle_in_radian * 180.0 / M_PI;

            return (float)angle_in_degree;
        }
        
        /*
        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
            depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            
            size_t margin = 15;
            size_t cnt = 0.0;
            float pixel_value = 0.0;
            float sum_pixel_value = 0.0;
            float degree = 0.0;
            uint32_t center_x = (int)((x1_+x2_)/2);
            uint32_t center_y = (int)((y1_+y2_)/2);

            for (size_t i = center_x-margin; i < center_x+margin; i++) {
                for (size_t j = center_y-margin; j < center_y+margin; j++) {

                    pixel_value = depth_image.at<float>(j, i) / 1000;

                    if (i == center_x && j == center_y)
                    {
                        degree = this->get_degree(center_x, center_y, pixel_value);
                    }

                    if (0.3 <= pixel_value && pixel_value <= 3.0)
                    {
                        sum_pixel_value += pixel_value;
                        cnt++;
                    }
                }
            }
            
            person_info.length = sum_pixel_value / cnt;
            person_info.degree = degree;

            person_info_pub_->publish(person_info);
        }

	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
	    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image;
    	    cv::rectangle(img, cv::Rect(cv::Point(x1_, y1_), cv::Point(x2_, y2_)), cv::Scalar(0, 0, 255), 1);
    	    
    	    sensor_msgs::msg::Image::SharedPtr msg_pub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    	    
    	    image_pub_->publish(*msg);
	}
	*/
	
	void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
              depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;

              size_t margin = 15;
              size_t cnt = 0.0;
              float pixel_value = 0.0;
              float sum_pixel_value = 0.0;
              float degree = 0.0;
              uint32_t center_x = (int)((x1_ + x2_) / 2);
              uint32_t center_y = (int)((y1_ + y2_) / 2);

              for (size_t i = center_x - margin; i < center_x + margin; i++) {
                  for (size_t j = center_y - margin; j < center_y + margin; j++) {
                      pixel_value = depth_image.at<float>(j, i) / 1000;

                      if (i == center_x && j == center_y) {
                          degree = this->get_degree(center_x, center_y, pixel_value);
                      }

                      if (0.3 <= pixel_value && pixel_value <= 3.0) {
                          sum_pixel_value += pixel_value;
                          cnt++;
                      }
                  }
              }

              person_info.length = sum_pixel_value / cnt;
              person_info.degree = degree;

              person_info_pub_->publish(person_info);
          } catch (const cv_bridge::Exception& e) {
              RCLCPP_ERROR(rclcpp::get_logger("depthCallback"), "cv_bridge exception: %s", e.what());
          } catch (const std::exception& e) {
              RCLCPP_ERROR(rclcpp::get_logger("depthCallback"), "Exception in depthCallback: %s", e.what());
          } catch (...) {
              RCLCPP_ERROR(rclcpp::get_logger("depthCallback"), "Unknown exception in depthCallback");
          }
      }

	
	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    	  try {
            cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image;
            cv::rectangle(img, cv::Rect(cv::Point(x1_, y1_), cv::Point(x2_, y2_)), cv::Scalar(255, 0, 0), 2);
	    
            sensor_msgs::msg::Image::SharedPtr msg_pub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
	    
            image_pub_->publish(*msg_pub);
    	  } catch (const cv_bridge::Exception& e) {
       	    RCLCPP_ERROR(rclcpp::get_logger("imageCallback"), "cv_bridge exception: %s", e.what());
    	  } catch (const std::exception& e) {
    	    RCLCPP_ERROR(rclcpp::get_logger("imageCallback"), "Exception in imageCallback: %s", e.what());
   	  } catch (...) {
   	    RCLCPP_ERROR(rclcpp::get_logger("imageCallback"), "Unknown exception in imageCallback");
  	  }
        }


        void bboxCallback(const result_msgs::msg::Bbox msg) {
            x1_ = msg.x1;
            y1_ = msg.y1;
            x2_ = msg.x2;
            y2_ = msg.y2;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<result_msgs::msg::Bbox>::SharedPtr bbox_sub_;
        

        rclcpp::Publisher<result_msgs::msg::Person>::SharedPtr person_info_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessingModule>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
