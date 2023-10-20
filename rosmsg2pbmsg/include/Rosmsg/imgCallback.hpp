#ifndef IMGCALLBACK_HPP
#define IMGCALLBACK_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/compressed_image.hpp"

class ImagedMsg : public rclcpp::Node
{
public:
        ImagedMsg();

protected:
        int imageFront_count = 0, imageBack_count = 0;
        uint16_t count_;

        void CompressedImageFront_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
        void CompressedImageBack_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
        void CompressedImage_mqtt_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
        // image-back
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr CompressedImageBack_sub_;

        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr CompressedImageBack_pub_;

        // image-front
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr CompressedImageFront_sub_;

        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr CompressedImageFront_pub_;
};

#endif