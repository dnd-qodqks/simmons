#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "result_msgs/msg/bbox.hpp"
#include "result_msgs/msg/person.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageProcessingModule : public rclcpp::Node {
    public:
        ImageProcessingModule() : Node("image_processing_node") {
            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/robot/D435/aligned_depth_to_color/image_raw", 10,
                std::bind(&ImageProcessingModule::depthCallback, this,
                            std::placeholders::_1));

            yolov8_sub_ = this->create_subscription<result_msgs::msg::Bbox>(
                "/yolo/bbox", 10, std::bind(&ImageProcessingModule::resultCallback, this,
                std::placeholders::_1));

            person_info_pub_ = this->create_publisher<result_msgs::msg::Person>(
                "/person_info", 10);
        }
    
    private:
        cv::Mat depth_image;
        uint32_t center_x = 0;
        uint32_t center_y = 0;
        result_msgs::msg::Person person_info;

        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
            depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;

            // int centerX = depth_image.cols / 2;
            // int centerY = depth_image.rows / 2;

            // float centralPixelValue = depth_image.at<float>(centerY, centerX) / 1000; // Unit: [m]

            // RCLCPP_INFO(this->get_logger(), "--[Receive Depth Image_raw) X: %d,  Y: %d, Depth: %lf]--", centerX, centerY, centralPixelValue);
            
            size_t margin = 15;
            size_t cnt = 0.0;
            float pixel_value = 0.0;
            float sum_pixel_value = 0.0;

            for (size_t i = center_x-margin; i < center_x+margin; i++) {
                for (size_t j = center_y-margin; j < center_y+margin; j++) {
                    pixel_value = depth_image.at<float>(j, i) / 1000;

                    if (0.3 <= pixel_value && pixel_value <= 3.0)
                    {
                        sum_pixel_value += pixel_value;
                        cnt++;
                    }
                }
            }
            
            person_info.length = sum_pixel_value / cnt;
            person_info.degree = 0.0;

            person_info_pub_->publish(person_info);
        }

        void resultCallback(const result_msgs::msg::Bbox msg) {
            center_x = msg.center_x;
            center_y = msg.center_y;

            // RCLCPP_INFO(this->get_logger(), "--[Receive Bbox]--");
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
        rclcpp::Subscription<result_msgs::msg::Bbox>::SharedPtr yolov8_sub_;

        rclcpp::Publisher<result_msgs::msg::Person>::SharedPtr person_info_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessingModule>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
