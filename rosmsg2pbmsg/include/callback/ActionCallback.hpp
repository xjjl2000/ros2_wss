#include"callback/CallbackFunctionInterface.hpp"
#include <mqtt/async_client.h>
#ifndef ACTIONCALLBACK_HPP
#define ACTIONCALLBACK_HPP

class FailureCallback:public CallbackFunctionInterfacePlus<const mqtt::token &>
{
        public:
        std::string  name;
        virtual void func(const mqtt::token &) override;
        FailureCallback(std::string name):name(name){};
};


class SuccessCallback:public CallbackFunctionInterfacePlus<const mqtt::token &>
{
        
        public:
        std::string name;
        virtual void func(const mqtt::token &) override;
        SuccessCallback(std::string name):name(name){};
};



#endif