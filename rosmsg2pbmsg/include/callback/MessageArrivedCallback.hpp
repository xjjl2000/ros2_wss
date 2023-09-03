
#include "callback/CallbackFunctionInterface.hpp"
#include <mqtt/callback.h>


#ifndef MESSAGEARRIVEDCALLBACK_HPP
#define MESSAGEARRIVEDCALLBACK_HPP

class PrintMessageArrivedCallback:public CallbackFunctionInterfacePlus<mqtt::const_message_ptr>
{
public:
  virtual void func(mqtt::const_message_ptr) override;
};

class PrintMessageSizeArrivedCallback:public CallbackFunctionInterfacePlus<mqtt::const_message_ptr>{
  public:
    virtual void func(mqtt::const_message_ptr) override;
};


#endif



