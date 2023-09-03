#include "callback/CallbackFunctionInterface.hpp"

void CallbackFunctionInterface::setConnOpts(mqtt::connect_options connOpts)
{
      this->connOpts = connOpts;
}
void CallbackFunctionInterface::setClient(std::shared_ptr<mqtt::async_client> client)
{
      this->client = client;
}
