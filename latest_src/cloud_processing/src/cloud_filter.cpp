#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "result_msgs/msg/bbox.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

template <typename T>
using shared_ref = std::shared_ptr<T>;

template <typename T, typename... Args>

constexpr shared_ref<T> CreateSharedRef(Args&&... args) {
  return std::make_shared<T>(std::forward<Args>(args)...);
}

class PointCloudProcessing {
 public:
  void FilteringVolxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_src,
                       pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_dst,
                       double x_voxel_size, double y_voxel_size,
                       double z_voxel_size) {

    // 포인트 클라우드가 비어 있는지 확인합니다.
    if (point_cloud_src->empty())
      return;

    static pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(point_cloud_src);
    voxel_filter.setLeafSize(x_voxel_size, y_voxel_size, z_voxel_size);
    voxel_filter.filter(point_cloud_dst);
  }

  void FilteringSOR(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_src,
                    pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_dst,
                    int num_neigbor_points, double std_multiplier) {
    
    // 포인트 클라우드가 비어 있는지 확인합니다.
    if (point_cloud_src->empty())
      return;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filter;
    sor_filter.setInputCloud(point_cloud_src);
    sor_filter.setMeanK(num_neigbor_points);
    sor_filter.setStddevMulThresh(std_multiplier);
    sor_filter.filter(point_cloud_dst);
  }

  void Image2PointcloudXY(uint32_t rgb_img_min_x, uint32_t rgb_img_min_y, 
                          uint32_t rgb_img_max_x, uint32_t rgb_img_max_y,
                          double& min_x, double& min_y, double& min_z,
                          double& max_x, double& max_y, double& max_z,
                          double&  center_x, double& center_y, double& center_z,
                          cv::Mat depth_image) {
    double c_x = 329.529296875;
    double c_y = 240.3614959716797;
    double f_x = 602.3125610351562;
    double f_y = 602.2564697265625;

    double depth_scale = 1000.0;
    int rgb_img_center_x = round((rgb_img_max_x+rgb_img_min_x)/2);
    int rgb_img_center_y = round((rgb_img_max_y+rgb_img_min_y)/2);

    min_z = depth_image.ptr<float>(rgb_img_min_y)[rgb_img_min_x] / depth_scale;
    max_z = depth_image.ptr<float>(rgb_img_max_y)[rgb_img_max_x] / depth_scale;
    center_z = depth_image.ptr<float>(rgb_img_center_y)[rgb_img_center_x] / depth_scale;

    if (min_z < -3.0) min_z = -3.0;
    if (max_z < -3.0) max_z = -3.0;
    if (center_z < -3.0) center_z = -3.0;
    if (min_z > 3.0) min_z = 3.0;
    if (max_z > 3.0) max_z = 3.0;
    if (center_z > 3.0) center_z = 3.0;

    min_x = min_z * ((double)rgb_img_min_x - c_x) / f_x;
    min_y = min_z * ((double)rgb_img_min_y - c_y) / f_y;
    max_x = max_z * ((double)rgb_img_max_x - c_x) / f_x;
    max_y = max_z * ((double)rgb_img_max_y - c_y) / f_y;
    center_x = center_z * ((double)rgb_img_center_x - c_x) / f_x;
    center_y = center_z * ((double)rgb_img_center_y - c_y) / f_y;
  }

  void GetRoiPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_src,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_roi,
      double& center_x, double& center_y, double& center_z,
      const double& margin=0.2) {

    size_t cloud_size = point_cloud_src->size();

    for (size_t i = 0; i < cloud_size; ++i) {
      pcl::PointXYZRGB& point = point_cloud_src->points[i];

      if (point.x >= center_x-margin && point.x <= center_x+margin && point.y >= center_y-margin &&
          point.y <= center_y+margin && point.z >= center_z-margin && point.z <= center_z+margin) {
        point_cloud_roi->push_back(point);
      }
    }
  }

  void VisualizationRectangle(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_src, float min_x,
      float min_y, float min_z, float max_x, float max_y, float max_z,
      int r = 255, int g = 0, int b = 255) {
    
    // 포인트 클라우드가 비어 있는지 확인합니다.
    if (point_cloud_src->empty())
      return;

    // 특정 영역의 포인트 색상 변경
    for (size_t i = 0; i < point_cloud_src->size(); ++i) {
      pcl::PointXYZRGB& point = point_cloud_src->points[i];

      if (point.x >= min_x && point.x <= max_x && point.y >= min_y &&
          point.y <= max_y && point.z >= min_z && point.z <= max_z) {
        point.r = r;
        point.g = g;
        point.b = b;
      
        // printf("point index[%lu]\n", i);
        // printf("color change point[%ld] x:%f y:%f z:%f\n", i, point.x,
        // point.y, point.z);
      }
    }
  }

  void VisualizationCircle(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_src,
    int r=200, int g=0, int b=200) {
      
    // 포인트 클라우드가 비어 있는지 확인합니다.
    if (point_cloud_src->empty())
      return;
    
    size_t cloud_size = point_cloud_src->size();
    for (size_t i = 0; i < cloud_size; ++i) {
      pcl::PointXYZRGB& point = point_cloud_src->points[i];

      point.r = r;
      point.g = g;
      point.b = b;
    }
  }

  // 특정 영역의 평균 거리와 방향을 계산하는 함수
  void CalculateProperties(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_src,
      double& average_distance, Eigen::Vector3f& direction) {

    // 포인트 클라우드가 비어 있는지 확인합니다.
    if (point_cloud_src->empty()) {
      average_distance = 0.0;
      direction = Eigen::Vector3f::Zero();  // 방향을 0 벡터로 설정합니다.
      return;
    }

    average_distance = 0.0;
    direction = Eigen::Vector3f::Zero();

    // 추출된 포인트 클라우드에서 평균 거리를 계산합니다.
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    double sum_distnace = 0.0;
    size_t cloud_size = point_cloud_src->size();
    for (size_t i = 0; i < cloud_size; i++) {
      pcl::PointXYZRGB& point = point_cloud_src->points[i];
      
      if (-3.0 < point.x && point.x < 3.0)
        sum_x += point.x;

      if (-3.0 < point.y && point.y < 3.0)
        sum_y += point.y;
      
      if (-3.0 < point.z && point.z < 3.0)
        sum_z += point.z;

      if (-3.0 < point.x && point.x < 3.0 && -3.0 < point.y && point.y < 3.0 &&
          -3.0 < point.z && point.z < 3.0)
        sum_distnace += sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }

    average_distance = sum_distnace / cloud_size;

    pcl::PointXYZ center_point;
    center_point.x = sum_x / cloud_size;
    center_point.y = sum_y / cloud_size;
    center_point.z = sum_z / cloud_size;

    // 대각선 길이 계산
    Eigen::Vector3f centroid_point(center_point.x, center_point.y,
                                   center_point.z);

    direction = centroid_point.normalized();
  }
};

class PointCloudModule : public rclcpp::Node {
 public:
  PointCloudModule() : Node("point_cloud_filter") {
    pc_processing_ = CreateSharedRef<PointCloudProcessing>();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/robot/D435/color/image_raw", 10,
      std::bind(&PointCloudModule::imageCallback, this,
                std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/robot/D435/aligned_depth_to_color/image_raw", 10,
      std::bind(&PointCloudModule::depthCallback, this,
                std::placeholders::_1));

    result_sub_ = this->create_subscription<result_msgs::msg::Bbox>(
      "/yolo/bbox", 10, std::bind(&PointCloudModule::resultCallback, this,
      std::placeholders::_1));

    processing_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/processing_image", 10);

    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_voxel_cloud", 10);
    
    sor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_sor_cloud", 10);
    
    roi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/roi_cloud", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker", 10);

    point_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("points_marker", 10);

    RCLCPP_INFO(this->get_logger(), "--[OK]--");
  }

 private:

  uint32_t rgb_img_min_x = 0;
  uint32_t rgb_img_min_y = 0;
  uint32_t rgb_img_max_x = 0;
  uint32_t rgb_img_max_y = 0;

  cv::Mat depth_image;
  cv::Mat img;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr image_msg;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_src(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud_src);

    // 복셀 필터 적용
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pc_processing_->FilteringVolxel(pcl_cloud_src, *ptr_filtered, 0.05, 0.05,
                                    0.05);

    // 윤곽선 제거 필터 적용
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pc_processing_->FilteringSOR(ptr_filtered, *output, 10, 1.0);

    // detect 영역(RGBD x,y) -> point cloud(x,y,z) 계산
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;


    pc_processing_->Image2PointcloudXY(rgb_img_min_x, rgb_img_min_y,
                                       rgb_img_max_x, rgb_img_max_y,
                                       min_x, min_y, min_z,
                                       max_x, max_y, max_z,
                                       center_x, center_y, center_z,
                                       depth_image);

    // RCLCPP_INFO(this->get_logger(), "rgb_img_min_x=%d, rgb_img_min_y=%d, rgb_img_max_x=%d, rgb_img_max_y=%d", rgb_img_min_x, rgb_img_min_y, rgb_img_max_x, rgb_img_max_y);
    // RCLCPP_INFO(this->get_logger(), "min_x=%lf, min_y=%lf, min_z=%lf, max_x=%lf, max_y=%lf, max_z=%lf", min_x, min_y, min_z, max_x, max_y, max_z);

    // ROI 설정
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_roi(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pc_processing_->GetRoiPointcloud(output, cloud_roi, center_x, center_y, center_z, 0.15);

    // 포인트 생성 -----------------------------------------------------------
    geometry_msgs::msg::Point point;
    point.x = center_x;
    point.y = center_y;
    point.z = center_z;

    // 마커 메시지 설정
    visualization_msgs::msg::Marker point_marker_msg;
    point_marker_msg.header.frame_id = msg->header.frame_id;
    point_marker_msg.header.stamp = this->now();
    point_marker_msg.type = visualization_msgs::msg::Marker::POINTS;
    point_marker_msg.action = visualization_msgs::msg::Marker::ADD;
    point_marker_msg.id = 0;
    point_marker_msg.scale.x = 0.05; // 포인트 크기
    point_marker_msg.scale.y = 0.05; // 포인트 크기
    point_marker_msg.color.r = 1.0; // 색상 (빨강)
    point_marker_msg.color.a = 1.0; // 투명도
    point_marker_msg.points.push_back(point); // 포인트 추가

    point_marker_pub_->publish(point_marker_msg);
    // -----------------------------------------------------------

    // bounding box 영역 표시
    // pc_processing_->VisualizationRectangle(output, min_x, min_y, 0.0, max_x,
    //                                        max_y, 2.0);

    // bbox -> 중심 circle 영역 표시
    pc_processing_->VisualizationCircle(cloud_roi);

    // 평균 거리 & 방향 계산
    double average_distance;
    Eigen::Vector3f direction;
    pc_processing_->CalculateProperties(cloud_roi, average_distance, direction);
    RCLCPP_INFO(this->get_logger(), "average_distance=%f, x=%f, y=%f, z=%f", 
              average_distance, direction.x(), direction.y(), direction.z());

    // 방향 시각화
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = msg->header.frame_id;  // 포즈의 좌표계 설정
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;  // 화살표의 길이
    // marker.scale.y = 0.0;   // 화살표의 너비
    // marker.scale.z = 0.0;   // 화살표의 높이
    marker.color.r = 1.0;   // 색상 설정
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;  // 투명도

    // 시작점 설정
    geometry_msgs::msg::Point start_point;
    start_point.x = 0.0;
    start_point.y = 0.0;
    start_point.z = 0.0;
    marker.points.push_back(start_point);

    // 끝점 설정
    geometry_msgs::msg::Point end_point;
    // end_point.x = direction.x() * average_distance;
    // end_point.y = direction.y() * average_distance;
    // end_point.z = direction.z() * average_distance;
    end_point.x = center_x;
    end_point.y = center_y;
    end_point.z = center_z;

    marker.points.push_back(end_point);

    // Marker 메시지 발행
    marker_pub_->publish(marker);

    // PCL -> ROSMsg 형변환
    auto voxel_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*ptr_filtered, *voxel_msg);
    voxel_msg->header = msg->header;
    voxel_pub_->publish(*voxel_msg);

    auto sor_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*output, *sor_msg);
    sor_msg->header = msg->header;
    sor_pub_->publish(*sor_msg);

    auto roi_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_roi, *roi_msg);
    roi_msg->header = msg->header;
    roi_pub_->publish(*roi_msg);

    // 결과 출력
    RCLCPP_INFO(this->get_logger(), "--[Publish Filtered Point Cloud Data]--");
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    int centerX = img.cols / 2;
    int centerY = img.rows / 2;

    cv::circle(img, cv::Point(centerX, centerY), 2, cv::Scalar(0, 0, 255), -1);

    image_msg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();

    processing_image_pub_->publish(*image_msg.get());

    // RCLCPP_INFO(this->get_logger(), "--[Pub Processing Image]--");
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;

    int centerX = depth_image.cols / 2;
    int centerY = depth_image.rows / 2;

    float centralPixelValue = depth_image.at<float>(centerY, centerX) / 1000; // Unit: [m]

    RCLCPP_INFO(this->get_logger(), "--[Receive Depth Image_raw) X: %d,  Y: %d, Depth: %lf]--", centerX, centerY, centralPixelValue);
  }

  void resultCallback(const result_msgs::msg::Bbox msg) {
    if (msg.min_x < 0 || msg.max_x < 0)
      return;

    rgb_img_min_x = msg.min_x;
    rgb_img_max_x = msg.max_x;
    rgb_img_min_y = msg.min_y;
    rgb_img_max_y = msg.max_y;

    RCLCPP_INFO(this->get_logger(), "--[Receive Bbox]--");
  }

  shared_ref<PointCloudProcessing> pc_processing_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<result_msgs::msg::Bbox>::SharedPtr result_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processing_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudModule>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
