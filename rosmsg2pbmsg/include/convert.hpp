#ifndef CONVERT_HPP
#define CONVERT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensors_msg.pb.h"
#include "pointcloud2.pb.h"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include "zlib.h"

#include "MyMqttClient.hpp"


class Convert : public rclcpp::Node
{

   public:
        Convert();

   private:
        int cloud_count=0,image=0;
        using PointField = sensor_msgs::msg::PointField;
        void pointcloud2_sub_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

        void image_f_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void image_b_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, MyMqttClient& mqtt_image_b);

        void Compressedimage_f_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
        void Compressedimage_b_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
        void Compressedimage_mqtt_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);


        void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

        void compress_data(const std::string& input, std::string& output);
        std::string base64_encode(const unsigned char* data, size_t length);

        uint16_t count_;
        //pointcloud2
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_;
        //image-back
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr Compressedimage_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

        rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr Compressedimage_pub_;

        //image-front
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr Compressedimage_f_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_f_sub_;

        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr Compressedimage_f_pub_;
        rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr image_f_pub_;
        //imu
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

        rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr imupb_pub_;

        MyMqttClient mqtt_image_pub;
        MyMqttClient mqtt_image_sub;

};

#endif  // MINIMAL_SUBSCRIBER_HPP