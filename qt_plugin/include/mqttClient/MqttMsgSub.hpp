#ifndef MQTTMSGSUB_HPP
#define MQTTMSGSUB_HPP

#include "mqttClient/MessageArrivedCallback.hpp"
#include "callback/ActionCallback.hpp"
#include "callback/CallbackFunctionInterface.hpp"
#include "callback/ConnectCallback.hpp"
#include "callback/DeliveryComplete.hpp"
#include "MyMqttClient.hpp"



class MqttSubscriber
{
        public:
        MqttSubscriber();

        void init();

        private:
                MyMqttClient mqttFrontImage_sub;
                MyMqttClient mqttBackImage_sub;
                MyMqttClient mqttImu_sub;
                MyMqttClient mqttLaserScan_sub;

};




#endif