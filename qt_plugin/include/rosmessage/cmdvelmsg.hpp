#ifndef CMDVELMSG_HPP
#define CMDVELMSG_HPP

#include "rclInclude.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVel : public rclcpp::Node
{
public:
        CmdVel();

private:
        uint16_t count_;

        

        // 实际线性速度、角速度订阅
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_A;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_B;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_V;

        void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
        void cmd_vel_callback_B(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
        void cmd_vel_callback_V(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
};

#endif