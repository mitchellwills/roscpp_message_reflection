#ifndef ROSCPP_MESSAGE_REFLECTION_NODE_HANDLE_H
#define ROSCPP_MESSAGE_REFLECTION_NODE_HANDLE_H

#include <string>
#include <ros/ros.h>
#include <roscpp_message_reflection/message_description_provider.h>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_impl.h>

namespace roscpp_message_reflection {

class MessageCreator {
public:
  MessageCreator(const MessageDescription::Ptr& description);
  boost::shared_ptr<Message> operator()();
private:
  const MessageDescription::Ptr description_;
};

class Publisher {
public:
  Publisher();
  Publisher(ros::Publisher pub, MessageDescription::Ptr description);

  operator bool() const;

  Message createMessage();
  MessageDescription::Ptr getMessageType();

  void publish(Message& msg);
  void publish(const boost::shared_ptr<const Message>& msg);

private:
  ros::Publisher pub_;
  MessageDescription::Ptr description_;
};

class Subscriber {
public:
  Subscriber();
  Subscriber(ros::Subscriber sub, MessageDescription::Ptr description);

  operator bool() const;

private:
  ros::Subscriber sub_;
  MessageDescription::Ptr description_;
};

class ServiceClient {
public:
  ServiceClient();
  ServiceClient(ros::ServiceClient client, ServiceDescription::Ptr description);

  operator bool() const;

  Message createRequestMessage();
  Message createResponseMessage();
  MessageDescription::Ptr getRequestType();
  MessageDescription::Ptr getResponseType();

  bool call(const Message& req, Message& res);

private:
  ros::ServiceClient client_;
  ServiceDescription::Ptr description_;
};

class ServiceServer {
public:
  ServiceServer();
  ServiceServer(ros::ServiceServer server, ServiceDescription::Ptr description);

  operator bool() const;

private:
  ros::ServiceServer server_;
  ServiceDescription::Ptr description_;
};

class NodeHandle {
public:
  NodeHandle(ros::NodeHandle& nh);

  Publisher advertise(const std::string& topic, const std::string& type);

  Subscriber subscribe(const std::string& topic, const std::string& type,
		       const boost::function< void(const boost::shared_ptr<const Message>&)> callback);

  ServiceServer advertiseService(const std::string& service, const std::string& type,
				 const boost::function<bool(const Message&, Message&)> callback);

  ServiceClient serviceClient(const std::string& service, const std::string& type);

private:
  ros::NodeHandle nh_;
  MessageDescriptionProvider::Ptr description_provider_;
};

}

#endif
