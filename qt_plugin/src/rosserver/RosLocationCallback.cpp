#include "rosserver/RosCallbackInterface.hpp"
#include "rosserver/DelayHz.hpp"
#include "udpserver/UdpSend.hpp"
#include "rclInclude.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

class RosPoseCallback : public RosCallbackInterface<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose::ConstSharedPtr>
{
        location pose;
        

        void callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg) override
        {
                pose.x = msg->position.x;
                pose.y = msg->position.y; 
        }

        public:
        RosPoseCallback(const std::string topic,rclcpp::Node* node) : RosCallbackInterface<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose::ConstSharedPtr>(topic, node){};

};

class RosPose2DCallback : public RosCallbackInterface<geometry_msgs::msg::Pose2D, geometry_msgs::msg::Pose2D::ConstSharedPtr>
{
        location pose;
        

        void callback(const geometry_msgs::msg::Pose2D::ConstSharedPtr msg) override
        {
                pose.x = msg->x;
                pose.y = msg->y; 
        }


        public:
        RosPose2DCallback(const std::string topic,rclcpp::Node* node) : RosCallbackInterface<geometry_msgs::msg::Pose2D, geometry_msgs::msg::Pose2D::ConstSharedPtr>(topic, node){};

};

class RosPoseStampedCallback : public RosCallbackInterface<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped::ConstSharedPtr>
{
        location pose;

        void callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) override
        {
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
        }


        public:
        RosPoseStampedCallback(const std::string topic,rclcpp::Node* node) : RosCallbackInterface<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped::ConstSharedPtr>(topic, node){};

};