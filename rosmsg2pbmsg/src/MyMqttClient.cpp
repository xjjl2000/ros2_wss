#include "MyMqttClient.hpp"

MyMqttClient::MyMqttClient(const std::string &name,
                   const std::string &host,
                   const int port,
                   mqtt::connect_options connOpts,
                   std::shared_ptr<CompentsCallback> callbackPtr,
                   std::shared_ptr<CompentsActionCallback> actionCallbackPtr)
    : host(host), port(port), connOpts(connOpts), name(name)
{
        // 设计为一个sub仅仅connect一次，所以connect直接在初始化函数完成
        client = std::make_shared<mqtt::async_client>(host + ":" + std::to_string(port), name);
        if (!callbackPtr)
                this->callbackPtr = std::make_shared<CompentsCallback>();
        else
                this->callbackPtr = callbackPtr;

        if (!actionCallbackPtr)
                this->actionCallbackPtr = std::make_shared<CompentsActionCallback>();
        else
                this->actionCallbackPtr = actionCallbackPtr;

        connect();
}

MyMqttClient::~MyMqttClient()
{
        // auto ptr = client.get();
        // ptr->disconnect()->wait();
}

int MyMqttClient::sub(const std::string &topic, int qos)
{

        auto cli = client.get();
        auto cbs_ptr = callbackPtr.get();
        // std::cout<<":start sub:"<<topic<<std::endl;

        if (!cli->is_connected())
        {
                // reconnct
                auto reconnect_callback_ptr = dynamic_cast<ReConnectConnectionLostCallback *>(callbackPtr->getConnectionLostFunc().get());
                if (reconnect_callback_ptr)
                {
                        reconnect_callback_ptr->reconnect();
                }
                else
                {
                        throw std::runtime_error("this function is not ReConnectConnectionLostCallback");
                }
        }

        cli->subscribe(topic, qos, nullptr, *actionCallbackPtr.get());
        // std::cout<<":end sub:"<<std::endl;
}

MyMqttClient::MyMqttClient() {}

void MyMqttClient::connect()
{
        // connect

        auto cac_ptr = actionCallbackPtr.get();
        auto cli = client.get();
        auto cb = callbackPtr.get();
        cb->setClient(client);
        cb->setConnOpts(connOpts);

        cli->set_callback(*cb);
        try
        {
                std::cout << "Connecting to the MQTT server..." << std::endl;
                // cli->connect(connOpts, nullptr, *cb)->wait();
                cli->connect(connOpts, nullptr, *cac_ptr)->wait();
                std::cout << "wait end" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        catch (const mqtt::exception &exc)
        {
                std::cerr << "\nERROR: Unable to connect to MQTT server: '"
                          << host << ":" << port << "'" << exc << std::endl;
                return;
        }
}


int MyMqttClient::pub(const std::string& topic,const std::string& message,int qos){
        auto ptr = client.get();
        std::cout << "\nSending message..." << std::endl;
        auto pubmsg = mqtt::make_message(topic, message);
        pubmsg->set_qos(qos);
        ptr->publish(pubmsg);
        std::cout << topic.c_str()<<"...OK" << std::endl;

        // lient.publish(TOPIC, PAYLOAD2, strlen(PAYLOAD2)+1);
}