#ifndef MQTT_SUB_DEF_H__
#define MQTT_SUB_DEF_H__

#include <iostream>
#include <mosquitto.h>
#include <signal.h>
#include <sstream>
#include <unistd.h>
#include <thread> 
#include <cstring>
#include <mutex>
#include <condition_variable>


        class Mqtt_sub
        {
                
                private:
                        
                        std::string _host;
                        int _port;
                        bool _connected=false;
                        struct mosquitto* _mosq = nullptr;
                        int _keep_alive = 60;
                        std::string _mesg_pub_topic = "mqttpublish";
                        static std::string _receivedmsg;
                        static Mqtt_sub* _instance;

                        static std::condition_variable _cv;
                        static bool _msgReceived; 
                        static std::mutex _mutex;     
                public:
                        Mqtt_sub(const std::string& host, const int port);
                        ~Mqtt_sub();

                        void init();
                        static int call_back_func(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg);        

                        std::string sub(const std::string& topic);        
        };


#endif