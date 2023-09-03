#ifndef MQTT_DEF_H__
#define MQTT_DEF_H__

#include <iostream>
#include <mosquitto.h>
#include <signal.h>
#include <sstream>
#include <unistd.h>


namespace rviz2_plugin
{
    class Mqtt_pub 
    {   
        private:
                std::string _host;
                int _port;
                struct mosquitto* _mosq = nullptr;
                int _keep_alive = 60;
                bool _connected = false;
                std::string _mesg_pub_topic = "mqttpublish";
        public:
                Mqtt_pub(const std::string& host, const int port);
                ~Mqtt_pub();

                void init();
                int pub_message(const std::string& _mesg_pub_topic, const std::string& message_string);
        


    };

}

#endif