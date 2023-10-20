#include "Rosmsg/imgCallback.hpp"
#include "sensors_msg.pb.h"

ImagedMsg::ImagedMsg() : Node("image_convert"), count_(0)
{
        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.keep_last(10);
    std::cout<<"CompressedImageBack_sub_ start"<<std::endl;

        CompressedImageBack_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/rear_camera_nx/camera/image_raw/compressed", 10, std::bind(&ImagedMsg::CompressedImageBack_callback, this, std::placeholders::_1));
        CompressedImageBack_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/rear_camera/camera/image_raw/compressed/convert", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        CompressedImageFront_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/d455_camera_nx/color/image_raw/compressed/convert", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
        CompressedImageFront_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/d455_camera_nx/color/image_raw/compressed", qos, std::bind(&ImagedMsg::CompressedImageFront_callback, this, std::placeholders::_1));
}

void ImagedMsg::CompressedImageFront_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
        imageFront_count++;
        std::cout<<"imageFront count:"<<imageFront_count<<std::endl;

        std::string str1 = msg->header.frame_id;
        std::string str2 = std::to_string(imageFront_count);
        std::string str3 = " ";
        std::string id = str1 + "" + str3 + "" + str2;

        struct timespec t = {0, 0};
        clock_gettime(CLOCK_REALTIME, &t);
        int t2 = t.tv_nsec;
        int t1 = t.tv_sec;

        sensor_msgs::msg::CompressedImage Compressedimage_msg;
        Compressedimage_msg.header.frame_id = id;
        Compressedimage_msg.header.stamp.nanosec = t2;
        Compressedimage_msg.header.stamp.sec = t1;
        Compressedimage_msg.set__format(msg->format);
        Compressedimage_msg.data.resize(msg->data.size());
        memcpy(Compressedimage_msg.data.data(), msg->data.data(), msg->data.size());

        CompressedImageFront_pub_->publish(Compressedimage_msg);
}
void ImagedMsg::CompressedImageBack_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
        imageBack_count++;
        std::cout<<"imageBack count:"<<imageBack_count<<std::endl;

        std::string str1 = msg->header.frame_id;
        std::string str2 = std::to_string(imageFront_count);
        std::string str3 = " ";
        std::string id = str1 + "" + str3 + "" + str2;
        struct timespec t = {0, 0};
        clock_gettime(CLOCK_REALTIME, &t);
        int t2 = t.tv_nsec;
        int t1 = t.tv_sec;

        sensor_msgs::msg::CompressedImage Compressedimage_msg;
        Compressedimage_msg.header.frame_id = id;
        Compressedimage_msg.header.stamp.nanosec = t2;
        Compressedimage_msg.header.stamp.sec = t1;
        Compressedimage_msg.set__format(msg->format);
        Compressedimage_msg.data.resize(msg->data.size());
        memcpy(Compressedimage_msg.data.data(), msg->data.data(), msg->data.size());
        CompressedImageBack_pub_->publish(Compressedimage_msg);
}
void ImagedMsg::CompressedImage_mqtt_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
        imageFront_count++;

        struct timespec t = {0, 0};
        clock_gettime(CLOCK_REALTIME, &t);
        int t2 = t.tv_nsec;
        int t1 = t.tv_sec;

        sensors_msg::ImageProto msg_pb;

        std::string str1 = msg->header.frame_id;
        std::string str2 = std::to_string(imageFront_count);
        std::string str3 = " ";
        std::string id = str1 + "" + str3 + "" + str2;

        msg_pb.set_header_frame_id(id);
        msg_pb.set_header_stamp_nanosec(t2);
        msg_pb.set_header_stamp_sec(t1);
        msg_pb.set_format(msg->format);
        msg_pb.set_data(msg->data.data(), msg->data.size());

        // base64
        // std::string base64_data = base64_encode(reinterpret_cast<const unsigned char *>(msg_pb.data().data()), msg_pb.data().size());

        std::string ser_msg;
        msg_pb.SerializeToString(&ser_msg);
        // std::cout << "format:" << msg->format.c_str() << std::endl;

        // std::string compressed_data;
        // compress_data(ser_msg, compressed_data);
        // std::cout << "ccccompressed_size:" << compressed_data.size() << std::endl;

        sensor_msgs::msg::CompressedImage imgc;
        imgc.format = msg->format;
        imgc.header.frame_id = id;
        imgc.header.stamp.nanosec = t2;
        imgc.header.stamp.sec = t1;
        imgc.data.resize(msg->data.size());
        memcpy(imgc.data.data(), msg->data.data(), msg->data.size());

        //   mqtt_image_pub.pub("mqtt_image", ser_msg, 0);
        CompressedImageBack_pub_->publish(imgc);
}
