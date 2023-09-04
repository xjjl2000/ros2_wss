
#include "callback/CallbackFunctionInterface.hpp"
#include <mqtt/callback.h>
#include <cv_bridge/cv_bridge.h>
#include <QWidget>
#include <QImage>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs.pb.h"
#include "sensors_msg.pb.h"
#include "pointcloud2.pb.h"



#ifndef MESSAGEARRIVEDCALLBACK_HPP
#define MESSAGEARRIVEDCALLBACK_HPP

class PrintMessageArrivedCallback:public CallbackFunctionInterfacePlus<mqtt::const_message_ptr>
{
public:
  virtual void func(mqtt::const_message_ptr) override;
  void dealCmdvelMessages(mqtt::const_message_ptr);
  void dealImageMessages(mqtt::const_message_ptr);
  void dealCloudMessages(mqtt::const_message_ptr);
  void dealImuMessages(mqtt::const_message_ptr);
  QImage convertToQImage(const cv::Mat& image);

};

class PrintMessageSizeArrivedCallback:public CallbackFunctionInterfacePlus<mqtt::const_message_ptr>{
  public:
    virtual void func(mqtt::const_message_ptr) override;
};


#endif



