#ifndef MSGDEL_HPP
#define MSGDEL_HPP

#include "rclInclude.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensors_msg.pb.h"
#include "QImage"
#include "qpixmap.h"




class ImageMsg : public rclcpp::Node
{
public:
  ImageMsg();

private:

  stamp_t image_At_f,image_At_b,image_Bt_f,image_Bt_b,image_Vt_f,image_Vt_b;
  struct timespec lastTime = {0, 0};
  struct timespec lastTime1 = {0, 0};
  struct timespec lastTime2 = {0, 0};
  struct timespec lastTime3 = {0, 0};

  void imagesub_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  
  void V_r_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  
  void A_f_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void B_f_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  
  void A_r_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void B_r_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  QImage convertToQImage(const cv::Mat& image);//senosor_msg转化成图片

  uint16_t count_;

  //前置
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_ACompressed;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_BCompressed;
  
  //后置
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_ARCompressed;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_BRCompressed;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_V;

};

#endif