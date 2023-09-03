#include"callback/CallbackFunctionInterface.hpp"
#include <string>
#include<mqtt/delivery_token.h>
#ifndef DELIVERYCOMPLETE_HPP
#define DELIVERYCOMPLETE_HPP

class PrintDeliveryCompleteCallback:public CallbackFunctionInterfacePlus<mqtt::delivery_token_ptr>
{
public:
  virtual void func(mqtt::delivery_token_ptr /*cause*/) override;
};


#endif