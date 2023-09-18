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
  QlabelShowMessageArrivedCallback(QLabel *label_=nullptr,QLabel *label1_=nullptr,QLabel *label2_=nullptr);

private:
  QLabel *label=nullptr;
  QLabel *label1=nullptr;
  QLabel *label2=nullptr;
  QMutex qMutex;

  struct timespec lastTime = {0, 0};
  typedef struct{
      int nanosec;
      int sec;
    }stamp_t;
  stamp_t image_Bt_b,cloud_t;
  int message_count = 0;
  struct timespec start_time;
  double delay=0;


  void dealCmdvelMessages(mqtt::const_message_ptr,QLabel *label,QLabel *label1);
  void dealImageMessages(mqtt::const_message_ptr,QLabel *label,QLabel *label1,QLabel *label2);
  void dealCloudMessages(mqtt::const_message_ptr,QLabel *label,QLabel *label1);
  void dealImuMessages(mqtt::const_message_ptr,QLabel *label,QLabel *label1,QLabel *label2);
  QImage convertToQImage(const cv::Mat& image);

  // signals:
  //   void updateLabel(QLabel* label,QPixmap & data);

};


#endif



