#include"callback/MessageArrivedCallback.hpp"


void PrintMessageSizeArrivedCallback::func(mqtt::const_message_ptr msg) {
  std::cout << "Message arrived" << std::endl;
  std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
  std::cout << "\tpayload_size: '" << msg->to_string().size() << "'\n"
            << std::endl;
}


void PrintMessageArrivedCallback::func(mqtt::const_message_ptr msg) {
  std::cout << "Message arrived" << std::endl;
  std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
  std::cout << "\tpayload: '" << msg->to_string() << "'\n"
            << std::endl;
}