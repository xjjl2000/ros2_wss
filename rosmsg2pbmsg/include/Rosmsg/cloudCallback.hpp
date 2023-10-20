#ifndef CLOUDCALLBACK_HPP
#define CLOUDCALLBACK_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pointcloud2.pb.h"

class PointCloudMsg : public rclcpp::Node
{
public:
        PointCloudMsg();

private:
        int cloud_count = 0, laserscan_count = 0;

        void pointcloud2_sub_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
        void laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

        // laserscan
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_sub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_pub;
};

#endif