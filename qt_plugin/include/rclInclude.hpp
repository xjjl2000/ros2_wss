
#ifndef RCLCPP_INCLUDE_HPP
#define RCLCPP_INCLUDE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include <fstream>
#include <iostream>
#include <chrono>
#include <Qt>
#include <qobject.h>
#include <QtNetwork/QUdpSocket>
#include "udpserver/UdpMessage.hpp"
#include <QPointF>
#include <QtCore/QVector>

using namespace std::chrono_literals;

typedef struct
{
      int nanosec;
      int sec;
} stamp_t;

typedef struct
{
      struct timespec image_f;
      struct timespec image_b;
      struct timespec imu;
      struct timespec pointcloud2;
} lasttime;



#endif