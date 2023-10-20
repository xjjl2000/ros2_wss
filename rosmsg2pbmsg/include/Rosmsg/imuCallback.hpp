#ifndef IMUCALLBACK_HPP
#define IMUCALLBACK_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/imu.hpp"

class ImuMsg : public rclcpp::Node
{
public:
        ImuMsg();

protected:
        int imucount = 0;
        void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

        // imu
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

#endif