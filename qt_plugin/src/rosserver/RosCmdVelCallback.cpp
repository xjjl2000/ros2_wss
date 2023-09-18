#include "rclInclude.hpp"
#include "rosserver/RosCallbackInterface.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RosCmdVelCallback : public RosCallbackInterface<geometry_msgs::msg::Twist, geometry_msgs::msg::Twist::ConstSharedPtr>
{

        void callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg) override
        {
                QString linear_x = QString::number(msg->linear.x, 'f', 2);
                QString angular_z = QString::number(msg->linear.x, 'f', 2);
        }

public:
        RosCmdVelCallback(const std::string topic, rclcpp::Node *node) : RosCallbackInterface<geometry_msgs::msg::Twist, geometry_msgs::msg::Twist::ConstSharedPtr>(topic, node){};
};