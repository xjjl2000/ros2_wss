
#ifndef CALLBACKFUCTIONINTERFACE_HPP
#define CALLBACKFUCTIONINTERFACE_HPP

#include<mqtt/async_client.h>
#include "qlabel.h"

class CallbackFunctionInterface:public QObject
{
private:
       mqtt::connect_options connOpts;
       std::shared_ptr<mqtt::async_client> client;
public:
      void setConnOpts(mqtt::connect_options connOpts);
      void setClient (std::shared_ptr<mqtt::async_client> );
        
};

template<class T>
class  CallbackFunctionInterfacePlus: public CallbackFunctionInterface{
        public:
        virtual void func(T /*cause*/){};
};

#endif