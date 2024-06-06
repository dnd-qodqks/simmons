#include "qnode.h"

qnode::qnode()
{
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("img_sub_node");

    mode_pub_ = node_->create_publisher<result_msgs::msg::Mode>(
                "/mode_info", 10);

    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/yolo/dbg_image", 10, std::bind(&qnode::imageCallback, this, std::placeholders::_1));

    force_sub_ = node_->create_subscription<result_msgs::msg::Force>(
            "force_info", 10, std::bind(&qnode::forceCallback, this, std::placeholders::_1));

    timer_ = node_->create_wall_timer(
                 std::chrono::milliseconds(10),
                 std::bind(&qnode::publish_message, this));

    mode_info.mode = 0;
}

void qnode::publish_message() 
{
  mode_pub_->publish(mode_info);
}

void qnode::set_image_label(QLabel* image_label)
{
    image_label_ = image_label;
}

void qnode::set_fps_label(QLabel* fps_label)
{
    fps_label_ = fps_label;
    fps_label_->setText("Mode 0");
}

void qnode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // qDebug() << "Received image, width:" << msg->width << ", height:" << msg->height;
    
    // qDebug() << "mode:" << mode_info.mode;

    if (mode_info.mode == 1)
    {
        QImage image(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
        QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, image), Q_ARG(quint32, mode_info.mode));
    }
}

void qnode::forceCallback(const result_msgs::msg::Force msg)
{
    qDebug() << "Force) x:" << msg.x << "y: " << msg.y;
 
    mode_pub_->publish(mode_info);

    if(mode_info.mode == 2)
    {
        cv::Mat img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);

        cv::Point start(320, 240);
        cv::Point end(500, 300);
        cv::arrowedLine(img, start, end, cv::Scalar(255, 0, 0), 3);

        QImage image = matToQImage(img);
        QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, image), Q_ARG(quint32, mode_info.mode));
    }
}

void qnode::updateImage(const QImage& image, const quint32 mode)
{
    // QLabel에 이미지 출력
    image_label_->setPixmap(QPixmap::fromImage(image));
    QString modeText = QString("Mode %1").arg(mode);
    fps_label_->setText(modeText);
}

QImage qnode::matToQImage(const cv::Mat& mat)
{
    // Mat이 비어있는지 확인
    if (mat.empty())
        return QImage();

    // Mat의 차원 확인
    switch (mat.type())
    {
        case CV_8UC1:
            return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_Grayscale8);
        case CV_8UC3:
            return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_RGB888);
        case CV_8UC4:
            return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_ARGB32);
        default:
            break;
    }

    return QImage();
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

    if (mode_pub_) {
        mode_pub_.reset();
    }

    if (image_sub_) {
        image_sub_.reset();
    }
}
