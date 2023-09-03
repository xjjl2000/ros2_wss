#include "my_plugin/mqtt_pub.hpp"
#include <iostream>
#include <signal.h>


namespace rviz2_plugin
{
        

        Mqtt_pub::Mqtt_pub(const std::string& host, const int port)
        : _host(host)
        , _port(port)
        {}

        

        Mqtt_pub::~Mqtt_pub() {
                if (_connected) {
                mosquitto_destroy(_mosq);
                }
        }


        void Mqtt_pub::init() {
                // init publish struct
                _mosq = mosquitto_new(nullptr, true, nullptr);
                if (mosquitto_connect(_mosq, _host.c_str(), _port, _keep_alive) != MOSQ_ERR_SUCCESS) {
                        std::cerr << "connect error!!"<< std::endl;
                } else {
                        _connected = true;
                        int rc = mosquitto_loop_start(_mosq);
                        if(rc != MOSQ_ERR_SUCCESS){
                                mosquitto_destroy(_mosq);
                                fprintf(stderr, "Error: %s\n", mosquitto_strerror(rc));
                                throw std::runtime_error("Error: !!!!");
                        }
                }
        }

        int Mqtt_pub::pub_message(const std::string& _mesg_pub_topic, const std::string& message_string) {
                

                int rc=mosquitto_publish(_mosq, NULL, _mesg_pub_topic.c_str(),
                                                message_string.size(),
                                                message_string.c_str(), 0, true);

                std::cout<<"rc:"<<rc<<std::endl;
                std::cout<<_mesg_pub_topic.c_str()<<" size:"<<message_string.length()<<std::endl;
                if (rc != MOSQ_ERR_SUCCESS) {

                        std::cerr  << "MQTT publish error." << std::endl;
                        return 1;
                }
                return 0;
        }

}
