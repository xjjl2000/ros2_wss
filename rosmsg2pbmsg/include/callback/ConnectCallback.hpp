#include"callback/CallbackFunctionInterface.hpp"
#include <string>
#include <memory>
#include "callback/CompentsActionCallback.hpp"

#include <mqtt/async_client.h>
#ifndef CONNECTCALLBACK_HPP
#define CONNECTCALLBACK_HPP

class PrintConnectionLostCallback:public CallbackFunctionInterfacePlus<const std::string &>
{
public:
  virtual void func(const std::string & /*cause*/) override;
};

class PrintConnectedCallback:public CallbackFunctionInterfacePlus<const std::string &>
{
public:
  virtual void func(const std::string & /*cause*/) override;
};

class ReConnectConnectionLostCallback:public CallbackFunctionInterfacePlus<const std::string &>
{
  private:
    std::shared_ptr<mqtt::async_client> cli; 
    mqtt::connect_options connOpts;//用于重连
    std::shared_ptr<CompentsActionCallback> actionCallbackPtr;
    
public:
  virtual void func(const std::string & /*cause*/) override;
    ReConnectConnectionLostCallback();
  ReConnectConnectionLostCallback(std::shared_ptr<CompentsActionCallback> actionCallbackPtr):
 actionCallbackPtr(actionCallbackPtr){};
      void reconnect();

};





#endif
