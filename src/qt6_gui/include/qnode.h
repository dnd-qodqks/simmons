#ifndef QNODE_H
#define QNODE_H

#include <QDebug>
#include <QImage>
#include <QLabel>
#include <QObject>
#include <QThread>
#include <QMetaObject>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "result_msgs/msg/mode.hpp"
#include "result_msgs/msg/force.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class qnode : public QThread
{
    Q_OBJECT
public:
    qnode();
    ~qnode() override;
    void run() override;
    result_msgs::msg::Mode mode_info;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<result_msgs::msg::Mode>::SharedPtr mode_pub_;
    rclcpp::Subscription<result_msgs::msg::Force>::SharedPtr force_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    QLabel* image_label_;
    QLabel* fps_label_;

public slots:
    void publish_message(void);
    void set_image_label(QLabel* image_label);
    void set_fps_label(QLabel* fps_label);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void forceCallback(const result_msgs::msg::Force msg);
    void updateImage(const QImage& image, const quint32 mode);
    QImage matToQImage(const cv::Mat& mat);
    
private:
signals:
};

#endif
