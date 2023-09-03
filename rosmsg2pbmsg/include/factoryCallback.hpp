#ifndef FACTORYCALLBACK_HPP
#define FACTORYCALLBACK_HPP
#include <QWidget>
#include "qpixmap.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MQTTClient.h"

typedef int (*callbackFunction)(void *context, char *topicName, int topicLen, MQTTClient_message *message);


callbackFunction factoryCallback(const std::string &topic);



#endif  