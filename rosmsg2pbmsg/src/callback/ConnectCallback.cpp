#include "callback/ConnectCallback.hpp"
#include <iostream>
#include <memory>
#include <thread>
#include "mqtt/async_client.h"

void PrintConnectionLostCallback::func(const std::string &cause)
{
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
                std::cout << "\tcause: " << cause << std::endl;
}

ReConnectConnectionLostCallback::ReConnectConnectionLostCallback() 
{
        actionCallbackPtr = std::make_shared<CompentsActionCallback>();
}

void ReConnectConnectionLostCallback::func(const std::string &cause)
{
        auto cli_ = cli.get();
        auto connOpts_ = connOpts;
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
                std::cout << "\tcause: " << cause << std::endl;

        std::cout << "Reconnecting..." << std::endl;
        // nretry_ = 0;

        reconnect(); 
        
}

void ReConnectConnectionLostCallback::reconnect()
{
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        try
        {
                cli.get()->connect(connOpts, nullptr, *actionCallbackPtr.get());
        }
        catch (const mqtt::exception &exc)
        {
                std::cerr << "Error: " << exc.what() << std::endl;
                exit(1);
        }
}

void PrintConnectedCallback::func(const std::string &cause)
{
        std::cout << "\nConnection success\n" << std::endl;
}
