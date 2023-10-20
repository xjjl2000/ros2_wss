#include "callback/MessageArrivedCallback.hpp"
#include <cv_bridge/cv_bridge.h>

#include "qlabel.h"
#include <QMutex>


#ifndef MYPLUGINMESSAGEARRIVEDCALLBACK_HPP
#define MYPLUGINMESSAGEARRIVEDCALLBACK_HPP


class QlabelShowMessageArrivedCallback:public CallbackFunctionInterfacePlus<mqtt::const_message_ptr>
{

public:
  virtual void func(mqtt::const_message_ptr) ;
  QlabelShowMessageArrivedCallback();

private:
  QMutex qMutex;

  struct timespec lastTime = {0, 0};
  typedef struct{
      int nanosec;
      int sec;
    }stamp_t;
  stamp_t image_Bt_b,cloud_t;

  void dealCmdvelMessages(mqtt::const_message_ptr);
  void dealFrontImageMessages(mqtt::const_message_ptr);
  void dealRearImageMessages(mqtt::const_message_ptr);
  void dealLaserScanMessages(mqtt::const_message_ptr);
  void dealCloudMessages(mqtt::const_message_ptr);
  void dealImuMessages(mqtt::const_message_ptr);
  QImage convertToQImage(const cv::Mat& image);

  // signals:
  //   void updateLabel(QLabel* label,QPixmap & data);

};


#endif



