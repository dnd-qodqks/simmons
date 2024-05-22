#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  initUi();

  commNode.set_image_label(ui->image_label);

  commNode.start();
}

void MainWindow::initUi()
{
  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
  
  setStatusBar(nullptr);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  rclcpp::shutdown();
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() { delete ui;}
