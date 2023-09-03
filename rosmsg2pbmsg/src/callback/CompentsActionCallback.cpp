#include "callback/CompentsActionCallback.hpp"

void CompentsActionCallback::on_failure(const mqtt::token &tok) 
{
        if(failureFunc)
                failureFunc.get()->func(tok);
}

void CompentsActionCallback::on_success(const mqtt::token &tok) 
{
        if(successFunc)
                successFunc.get()->func(tok);
}

void CompentsActionCallback::setFailureFunc(std::shared_ptr<ActionCallbackType> func)
{
        failureFunc = func;
}
void CompentsActionCallback::setSuccessFunc(std::shared_ptr<ActionCallbackType> func)
{
        successFunc = func;
}
