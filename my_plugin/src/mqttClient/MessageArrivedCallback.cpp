#include"my_plugin/mqttClient/MessageArrivedCallback.hpp"
#include <QWidget>
#include <QImage>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs.pb.h"
#include "sensors_msg.pb.h"
#include "pointcloud2.pb.h"


QlabelShowMessageArrivedCallback::QlabelShowMessageArrivedCallback(QLabel *label_,QLabel *label1_){
    if(label_==nullptr || label1_==nullptr)
      throw std::runtime_error("you use a nullprt");
    this->label=label_;
    this->label1=label1_;
}

void QlabelShowMessageArrivedCallback::func(mqtt::const_message_ptr msg) {
  if (!strcmp(msg->get_topic().c_str(), "mqtt_cmd_vel")) {
    dealCmdvelMessages(msg,label);
  }

  if (!strcmp(msg->get_topic().c_str(), "mqtt_image")) {
    dealImageMessages(msg,label,label1);
  }

  if (!strcmp(msg->get_topic().c_str(), "mqtt_pointcloud2")) {
    
  }
}



void QlabelShowMessageArrivedCallback::dealCmdvelMessages(mqtt::const_message_ptr msg,QLabel *label){
  geometry_msg::Twist twist_msg;
  
  twist_msg.ParseFromArray(msg->get_payload().c_str(),msg->to_string().size());
  std::cout << "FROM topic: " << msg->get_topic() << "  msg:" << msg->get_payload().c_str() << std::endl;
  std::cout << "twist_msg: linear_x" << twist_msg.linear_x() << "  linear_y " << twist_msg.linear_y() << " size:" << twist_msg.ByteSize() << std::endl;
}

void QlabelShowMessageArrivedCallback::dealImageMessages(mqtt::const_message_ptr msg,QLabel *label,QLabel *label1){
  sensors_msg::ImageProto msg_pb;
  msg_pb.ParseFromString(msg->get_payload());
  std::cout << "FROM topic: " << msg->get_topic() << "  msg_size:" << msg->get_payload().size() <<"  msg_pb size:"<<msg_pb.data().size()<< std::endl;

  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t0=t.tv_sec,t1=t.tv_nsec;
  sensor_msgs::msg::CompressedImage imgc;
  imgc.set__format(msg_pb.format());
  imgc.header.frame_id=msg_pb.header_frame_id();
  imgc.header.stamp.nanosec=msg_pb.header_stamp_nanosec();
  imgc.header.stamp.sec=msg_pb.header_stamp_sec();
  imgc.data.resize(msg_pb.data().size());
  memcpy(imgc.data.data(),msg_pb.data().data(),msg_pb.data().size());

  try
  {
      cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(imgc,sensor_msgs::image_encodings::BGR8);
      cv::Mat imgCallback;
      imgCallback = cv_ptr_compressed->image;
      QImage img = convertToQImage(imgCallback);
      label->setPixmap(QPixmap::fromImage(img).scaled(label->width(),label->height()));
      // ui_->label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back->width(),ui_->label_image_back->height()));
      
  }
catch (cv_bridge::Exception& e)
  {
  //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
}

void QlabelShowMessageArrivedCallback::dealCloudMessages(mqtt::const_message_ptr,QLabel *label){}
  
  
void QlabelShowMessageArrivedCallback::dealImuMessages(mqtt::const_message_ptr,QLabel *label){}


QImage QlabelShowMessageArrivedCallback::convertToQImage(const cv::Mat& image) {
        cv::Mat rgb;
        if (image.channels() == 3) {
            // cvt Mat BGR 2 QImage RGB
            cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        } else {
            rgb = image.clone();
        }

        QImage img(rgb.cols, rgb.rows, QImage::Format_RGB888);
        uchar* imageData = img.bits();  // 获取图像数据指针

        // 将像素数据复制到QImage中
        memcpy(imageData, rgb.data, rgb.cols * rgb.rows * 3);  // 假设每个像素有三个通道

        return img;
}