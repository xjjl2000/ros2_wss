#include "callback/MessageArrivedCallback.hpp"
#include <cv_bridge/cv_bridge.h>

#include "qlabel.h"


#ifndef MYPLUGINMESSAGEARRIVEDCALLBACK_HPP
#define MYPLUGINMESSAGEARRIVEDCALLBACK_HPP


class QlabelShowMessageArrivedCallback:public CallbackFunctionInterfacePlus<mqtt::const_message_ptr>
{

public:
  virtual void func(mqtt::const_message_ptr) ;
  QlabelShowMessageArrivedCallback(QLabel *label_=nullptr,QLabel *label1_=nullptr);

private:
  QLabel *label=nullptr;
  QLabel *label1=nullptr;

  void dealCmdvelMessages(mqtt::const_message_ptr,QLabel *label);
  void dealImageMessages(mqtt::const_message_ptr,QLabel *label,QLabel *label1);
  void dealCloudMessages(mqtt::const_message_ptr,QLabel *label);
  void dealImuMessages(mqtt::const_message_ptr,QLabel *label);
  QImage convertToQImage(const cv::Mat& image);

};


#endif



