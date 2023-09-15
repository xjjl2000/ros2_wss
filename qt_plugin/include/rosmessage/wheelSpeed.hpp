#ifndef WHEELSPEED_HPP
#define WHEELSPEED_HPP

#include "rclInclude.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class WheelSpeed : public rclcpp::Node
{
public:
        WheelSpeed();

private:
        typedef struct
        {
                double fl;
                double fr;
                double rl;
                double rr;
        } wheel_npm;

        uint16_t count_;

        void jointState_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
        void speedA(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void speedB(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void speedV(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

        // 实际轮速订阅
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nmp_A;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nmp_B;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nmp_V;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_V;
};

#endif