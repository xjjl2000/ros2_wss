#include "my_plugin/mqtt_sub.hpp"
#include <iostream>
#include <signal.h>

        Mqtt_sub::Mqtt_sub(const std::string& host, const int port)
        : _host(host)
        , _port(port)
        {_instance=this;}

        Mqtt_sub::~Mqtt_sub() {
                if (_connected) {
                mosquitto_lib_cleanup();
                mosquitto_destroy(_mosq);
                }
        }

        Mqtt_sub* Mqtt_sub::_instance = nullptr;
        std::string Mqtt_sub::_receivedmsg = "";
        bool Mqtt_sub::_msgReceived=false;
        std::mutex Mqtt_sub::_mutex;
        std::condition_variable Mqtt_sub::_cv;

        void Mqtt_sub::init() {
                // 在Mqtt_sub::init函数中初始化_msgReceived
                _msgReceived = false;
                // init publish struct
                _mosq = mosquitto_new(nullptr, true, nullptr);
                if (mosquitto_connect(_mosq, _host.c_str(), _port, _keep_alive) != MOSQ_ERR_SUCCESS) {
                std::cerr << "connect error!!" << std::endl;
                _connected = false;
                exit(-1);
                } else {
                _connected = true;
                }
                std::cout << "publish init finished" << std::endl;

                // init recive callback
                mosquitto_lib_init(); 

        }

        std::string Mqtt_sub::sub(const std::string& topic) {

                auto register_callback = [=] {
                mosquitto_subscribe_callback(
                        &Mqtt_sub::call_back_func, NULL,
                        topic.c_str(), 0,
                        _host.c_str(), _port,
                        NULL, 60, true,
                        NULL, NULL,
                        NULL, NULL);
                // this api can not return??
                };
                std::thread register_callback_thread(register_callback);
                register_callback_thread.detach();
                
                //在Mqtt_sub类中添加一个std::mutex对象作为同步锁，使用std::unique_lock来锁定锁对象。
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [&]() { return _msgReceived; });

                std::string message = Mqtt_sub::_receivedmsg;
                _msgReceived = false;

                std::cout<<"std::string callback"<<Mqtt_sub::_receivedmsg.c_str()<<std::endl;



                return message;
        }

        int Mqtt_sub::call_back_func(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg) {
                Mqtt_sub::_receivedmsg=(char*)msg->payload;
                Mqtt_sub::_msgReceived = true;
                Mqtt_sub::_cv.notify_one();
                std::cout<<"int callback"<<Mqtt_sub::_receivedmsg.c_str()<<std::endl;
                std::cout << "FROM topic: " << msg->topic <<"  msg:"<<(char *)msg->payload<< std::endl;
                
                // message.ParseFromArray((const char *)msg->payload, msg->payloadlen);
                // std::cout << "GOT message:\n" << message.DebugString();
                return 0;
        }
