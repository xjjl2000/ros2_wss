#include "callback/CallbackFunctionInterface.hpp"
#include <string>
#include <mqtt/delivery_token.h>
#include"callback/DeliveryComplete.hpp"

using namespace std;

void PrintDeliveryCompleteCallback::func(mqtt::delivery_token_ptr tok)
{
  cout << "\tDelivery complete for token: "
       << (tok ? tok->get_message_id() : -1) << endl;
}
