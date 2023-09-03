#ifndef MY_MQTT_CLIENT_H__
#define MY_MQTT_CLIENT_H__

#include <iostream>
#include "mqtt/async_client.h"
#include <signal.h>
#include <sstream>
#include <unistd.h>
#include <thread>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include "geometry_msgs.pb.h"
#include "callback/CompentsCallback.hpp"
#include "callback/CompentsActionCallback.hpp"

class MyMqttClient
{

private:
        std::string host, name;
        int port;
        mqtt::connect_options connOpts;
        std::shared_ptr<mqtt::async_client> client;
        std::shared_ptr<CompentsCallback> callbackPtr;
        std::shared_ptr<CompentsActionCallback> actionCallbackPtr;
        void connect();

public:
        MyMqttClient();
        MyMqttClient(
                const std::string &name,
                const std::string &host,
                const int port, 
                mqtt::connect_options connOpts,
                std::shared_ptr<CompentsCallback> callbackPtr=nullptr,
                std::shared_ptr<CompentsActionCallback> actionCallbackPtr=nullptr
                 
                );
        ~MyMqttClient();
        int sub(const std::string &topic, int qos);
        int pub(const std::string& topic,const std::string& message,int qos);
        
        


};

#endif