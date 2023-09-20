
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
#include <QMutex>


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

typedef struct
{
      float x;
      float y;
      float z;
} xyz;
typedef struct
{
      xyz Acc;
      xyz Angual;
      xyz Mag;
} IMU_msg;
typedef struct
{
      double x;
      double y;
} location;

typedef struct
{
      double fl;
      double fr;
      double rl;
      double rr;
} wheel_npm;

#endif