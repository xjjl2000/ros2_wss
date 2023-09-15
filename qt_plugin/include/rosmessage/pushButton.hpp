#ifndef PUSHBUTTON_HPP
#define PUSHBUTTON_HPP

#include "rclInclude.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class PushButton : public rclcpp::Node
{
public:
        PushButton();

private:
        uint16_t count_;
        
        double k, lfc, max_speed, max_omega;
        // 远程控制
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_pub_A;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_pub_B;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_pub_V;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr par_V;

        void controller_parm_button_clicked();

        void vedit_button_clicked();
        void vedit_button_clicked_B();
        void vedit_button_clicked_V();
        void up_button_clicked();
        void down_button_clicked();
        void left_button_clicked();
        void right_button_clicked();
        void stop_button_clicked();

        void up_button_clicked_V();
        void down_button_clicked_V();
        void left_button_clicked_V();
        void right_button_clicked_V();
        void stop_button_clicked_V();

        void up_button_clicked_B();
        void down_button_clicked_B();
        void left_button_clicked_B();
        void right_button_clicked_B();
        void stop_button_clicked_B();

        void up_(geometry_msgs::msg::Twist &msg);
        void down_(geometry_msgs::msg::Twist &msg);
        void left_(geometry_msgs::msg::Twist &msg);
        void right_(geometry_msgs::msg::Twist &msg);
        void stop_(geometry_msgs::msg::Twist &msg);

        void up_B(geometry_msgs::msg::Twist &msg);
        void down_B(geometry_msgs::msg::Twist &msg);
        void left_B(geometry_msgs::msg::Twist &msg);
        void right_B(geometry_msgs::msg::Twist &msg);
        void stop_B(geometry_msgs::msg::Twist &msg);

        void up_V(geometry_msgs::msg::Twist &msg);
        void down_V(geometry_msgs::msg::Twist &msg);
        void left_V(geometry_msgs::msg::Twist &msg);
        void right_V(geometry_msgs::msg::Twist &msg);
        void stop_V(geometry_msgs::msg::Twist &msg);

        void reset_button_clicked();
        // void points_cleart_button_clicked();在前端处理
};

#endif