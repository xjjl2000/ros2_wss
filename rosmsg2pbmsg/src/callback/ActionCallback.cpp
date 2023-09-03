#include"callback/ActionCallback.hpp"


 void FailureCallback::func(const mqtt::token &tok ){
      std::cout << name << " failure";
        if (tok.get_message_id() != 0)
                std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
        std::cout << std::endl;
 }



void SuccessCallback::func(const mqtt::token & tok) {
      std::cout << name << " success";
      if (tok.get_message_id() != 0)
              std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
      auto top = tok.get_topics();
      if (top && !top->empty())
              std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
      std::cout << std::endl;
}




