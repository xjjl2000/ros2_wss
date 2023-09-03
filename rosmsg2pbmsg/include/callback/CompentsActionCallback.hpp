#include <mqtt/callback.h>
#include "callback/CallbackFunctionInterface.hpp"
typedef CallbackFunctionInterfacePlus<const mqtt::token &> ActionCallbackType;

#ifndef COMPENTSACTIONCALLBACK_HPP
#define COMPENTSACTIONCALLBACK_HPP

class CompentsActionCallback:  public virtual  mqtt::iaction_listener{
        
        private:
        std::shared_ptr<ActionCallbackType> failureFunc;
        std::shared_ptr<ActionCallbackType> successFunc;
        

  
        public:

        void on_failure(const mqtt::token &tok) override;

        void on_success(const mqtt::token &tok) override;


	void setFailureFunc(std::shared_ptr<ActionCallbackType> func);
	void setSuccessFunc(std::shared_ptr<ActionCallbackType> func);
	
	


};


#endif