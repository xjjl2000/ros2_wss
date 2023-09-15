#ifndef LOCATION_HPP
#define LOCATION_HPP

#include "rclInclude.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

class LocationMsg : public rclcpp::Node
{
public:
        LocationMsg();

private:
        uint16_t count_;
        
        typedef struct
        {
                double x;
                double y;
        } location;

        location poseA, poseB;

        // april_tag
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point_B;

        // uwb
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr point_uwb;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr point_ubw_B;

        void uwb_poseA_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg);
        void uwb_poseB_callback(const geometry_msgs::msg::Pose2D::ConstSharedPtr msg);

        void poseA_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void poseB_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
};

#endif