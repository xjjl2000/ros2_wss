#include "callback/CompentsCallback.hpp"

void CompentsCallback::connected(const std::string &cause)
{
	if (connectedFunc)
		connectedFunc.get()->func(cause);
}
void CompentsCallback::connection_lost(const std::string &cause)
{
	if (connectionLostFunc)
		connectionLostFunc.get()->func(cause);
}
void CompentsCallback::message_arrived(mqtt::const_message_ptr msg)
{
	if (messageArrivedFunc)
		messageArrivedFunc.get()->func(msg);
}
void CompentsCallback::delivery_complete(mqtt::delivery_token_ptr token)
{
	if (deliveryCompleteFunc)
		deliveryCompleteFunc.get()->func(token);
}

void CompentsCallback::setConnectedFunc(std::shared_ptr<ConnectedFuncType> func)
{
	connectedFunc = func;
}
void CompentsCallback::setConnectionLostFunc(std::shared_ptr<connectionLostFuncType> func)
{
	connectionLostFunc = func;
}
void CompentsCallback::setMessageArrivedFunc(std::shared_ptr<MessageArrivedFuncType> func)
{
	messageArrivedFunc = func;
}
void CompentsCallback::setDeliveryCompleteFunc(std::shared_ptr<DeliveryCompleteFuncType> func)
{
	deliveryCompleteFunc = func;
}

std::shared_ptr<ConnectedFuncType> &CompentsCallback::getConnectedFunc()
{
	return connectedFunc;
}
std::shared_ptr<connectionLostFuncType> &CompentsCallback::getConnectionLostFunc()
{
	return connectionLostFunc;
}
std::shared_ptr<MessageArrivedFuncType> &CompentsCallback::setMessageArrivedFunc()
{
	return messageArrivedFunc;
}
std::shared_ptr<DeliveryCompleteFuncType> &CompentsCallback::setDeliveryCompleteFunc()
{
	return deliveryCompleteFunc;
}

void CompentsCallback::setConnOpts(mqtt::connect_options connOpts)
{
	this->connOpts = connOpts;
	if (connectedFunc)
	{
		connectedFunc.get()->setConnOpts(connOpts);
	}
	if (connectionLostFunc)
	{
		connectionLostFunc.get()->setConnOpts(connOpts);
	}
	if (messageArrivedFunc)
	{
		messageArrivedFunc.get()->setConnOpts(connOpts);
	}
	if (deliveryCompleteFunc)
	{
		deliveryCompleteFunc.get()->setConnOpts(connOpts);
	}
}
void CompentsCallback::setClient(std::shared_ptr<mqtt::async_client> client_)
{
	if(client_==nullptr)
		throw std::runtime_error("nullptr");
	client = client_;
	if (connectedFunc)
	{
		connectedFunc.get()->setClient(client);
	}
	if (connectionLostFunc)
	{
		connectionLostFunc.get()->setClient(client);
	}
	if (messageArrivedFunc)
	{
		messageArrivedFunc.get()->setClient(client);
	}
	if (deliveryCompleteFunc)
	{
		deliveryCompleteFunc.get()->setClient(client);
	}
}
