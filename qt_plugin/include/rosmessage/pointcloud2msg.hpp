#ifndef CLOUD2_HPP
#define CLOUD2_HPP

#include "rclInclude.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vtkRenderWindow.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

class PointCloudMsg : public rclcpp::Node
{
public:
  PointCloudMsg();

private:
  stamp_t cloud_At, cloud_Bt;
  struct timespec lastTime = {0, 0};
  struct timespec lastTime1 = {0, 0};

  void pointcloud2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);


  uint16_t count_;

  // 点云
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_A;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_B;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser;


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pb_2_cloud2_A;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pb_2_cloud2_B;
};

#endif