#ifndef IMUMSG_HPP
#define IMUMSG_HPP

#include "rclInclude.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class ImuMsg : public rclcpp::Node
{
public:
  ImuMsg();

private:
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

  IMU_msg imu_A, imu_B;
  stamp_t imu_At,imu_Bt;
  struct timespec lastTime = {0, 0};
  struct timespec lastTime1 = {0, 0};
  struct timespec lastTime2 = {0, 0};

  void imusub_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  void imudata_callback(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line);
  void imumag_callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line);

  void imudata_callback_B(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line);
  void imumag_callback_B(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line);
  uint16_t count_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  // IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr actualDataLine_sub_A;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr actualMag_data_sub_A;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr actualDataLine_sub_B;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr actualMag_data_sub_B;
};

#endif