#include "rosmessage/pointcloud2msg.hpp"

PointCloudMsg::PointCloudMsg() : Node("cloud2"), count_(0)
{

  pointcloud2_sub_A = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidarrr", 10, std::bind(&PointCloudMsg::pointcloud2_callback, this, std::placeholders::_1));
  
  laser=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&PointCloudMsg::scan_callback,this,std::placeholders::_1));
  
  pb_2_cloud2_A = this->create_publisher<sensor_msgs::msg::PointCloud2>("pb_2_Pointcloud2", 10);
  pb_2_cloud2_B = this->create_publisher<sensor_msgs::msg::PointCloud2>("pb_2_Pointcloud2_B", 10);
}

void PointCloudMsg::pointcloud2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg)
{

  this->cloud_At.sec = pointcloud2_msg->header.stamp.sec;
  this->cloud_At.nanosec = pointcloud2_msg->header.stamp.nanosec;

  if (this->cloud_At.nanosec)
  {
    struct timespec pC2_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &pC2_t);
    double time = pointcloud2_msg->header.stamp.sec + pointcloud2_msg->header.stamp.nanosec * 1e-9;
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / (pC2_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (pC2_t.tv_sec - this->lastTime.tv_sec);
      QString HZ=QString::number(hz,'1',1);
      
      // this->ui_->cloudHz_label->setText(QString::number(hz,'1',1));
    }
    this->lastTime = pC2_t;

    int t0 = pC2_t.tv_sec;
    int t1 = pC2_t.tv_nsec;
    double pC2_delayt = (t0 + t1 * 1e-9) - (pointcloud2_msg->header.stamp.sec + pointcloud2_msg->header.stamp.nanosec * 1e-9);
    QString delayt=QString::number(pC2_delayt,'f',3);
    
    // this->ui_->cloudTime_label->setText(QString::number(pC2_delayt,'f',3));
  }
}

void PointCloudMsg::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg){

}