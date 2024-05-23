#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  initUi();

  QMetaObject::connectSlotsByName(this);

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

void MainWindow::on_Color_clicked()
{
  // qDebug() << "on_Color_clicked()";
  commNode.mode = 1;
}

void MainWindow::on_Depth_clicked()
{
  // qDebug() << "on_Depth_clicked()";
  commNode.mode = 2;
}

void MainWindow::on_Quit_clicked()
{
  this->close();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  rclcpp::shutdown();
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() { delete ui;}
