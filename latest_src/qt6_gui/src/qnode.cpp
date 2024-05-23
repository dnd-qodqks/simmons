#include "qnode.h"
#include <sensor_msgs/msg/image.hpp>

qnode::qnode()
{
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("img_sub_node");

    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/yolo/dbg_image", 10, std::bind(&qnode::imageCallback, this, std::placeholders::_1));
}

void qnode::set_image_label(QLabel* image_label)
{
    image_label_ = image_label;
}

void qnode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // qDebug() << "Received image, width:" << msg->width << ", height:" << msg->height;
    
    // qDebug() << "mode:" << mode;

    if (mode == 1)
    {
        QImage image(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
        QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, image));
    }
}

void qnode::updateImage(const QImage& image)
{
    // QLabel에 이미지 출력
    image_label_->setPixmap(QPixmap::fromImage(image));
}

void qnode::run()
{
    // qDebug() << "qnode start!";

    rclcpp::spin(node_);
    rclcpp::shutdown();

    qDebug() << "qnode shutdown!";
}

qnode::~qnode()
{
    if (image_label_) {
        delete image_label_;
        image_label_ = nullptr;
    }

    if (node_) {
        node_.reset();
    }

    if (image_sub_) {
        image_sub_.reset();
    }
}