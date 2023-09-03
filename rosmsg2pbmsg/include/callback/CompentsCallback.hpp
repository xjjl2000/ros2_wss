#include <mqtt/callback.h>
#include "callback/ConnectCallback.hpp"


#ifndef COMPENTSCALLBACK_HPP
#define COMPENTSCALLBACK_HPP
typedef CallbackFunctionInterfacePlus<const std::string &> ConnectedFuncType;
typedef CallbackFunctionInterfacePlus<const std::string &> connectionLostFuncType;
typedef CallbackFunctionInterfacePlus<mqtt::const_message_ptr> MessageArrivedFuncType;
typedef CallbackFunctionInterfacePlus<mqtt::delivery_token_ptr>  DeliveryCompleteFuncType;


class CompentsCallback: public mqtt::callback{
        private:
        std::shared_ptr<ConnectedFuncType> connectedFunc;
        std::shared_ptr<connectionLostFuncType > connectionLostFunc;
        std::shared_ptr<MessageArrivedFuncType > messageArrivedFunc;
        std::shared_ptr<DeliveryCompleteFuncType > deliveryCompleteFunc;
	mqtt::connect_options connOpts;
       std::shared_ptr<mqtt::async_client> client=nullptr;

  
        public:

        virtual void connected(const std::string& /*cause*/) override;
	/**
	 * This method is called when the connection to the server is lost.
	 */
	virtual void connection_lost(const std::string& /*cause*/) override;
	/**
	 * This method is called when a message arrives from the server.
	 */
	virtual void message_arrived(mqtt::const_message_ptr /*msg*/)  override;
	/**
	 * Called when delivery for a message has been completed, and all
	 * acknowledgments have been received.
	 */
	virtual void delivery_complete(mqtt::delivery_token_ptr /*tok*/)  override;
	void setConnectedFunc(std::shared_ptr<ConnectedFuncType> func);
	void setConnectionLostFunc(std::shared_ptr<connectionLostFuncType > func);
	void setMessageArrivedFunc(std::shared_ptr<MessageArrivedFuncType > func);
	void setDeliveryCompleteFunc(std::shared_ptr<DeliveryCompleteFuncType >  func);
	std::shared_ptr<ConnectedFuncType>& getConnectedFunc();
	std::shared_ptr<connectionLostFuncType >& getConnectionLostFunc();
	std::shared_ptr<MessageArrivedFuncType >& setMessageArrivedFunc();
	std::shared_ptr<DeliveryCompleteFuncType > & setDeliveryCompleteFunc();

	void setConnOpts(mqtt::connect_options connOpts);
        void setClient (std::shared_ptr<mqtt::async_client> );

	


};


#endif