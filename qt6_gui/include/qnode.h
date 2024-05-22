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

class qnode : public QThread
{
    Q_OBJECT
public:
    qnode();
    ~qnode() override;
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    QLabel* image_label_;

public slots:
    void set_image_label(QLabel* image_label);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void updateImage(const QImage& image);
    
private:
signals:
};

#endif
