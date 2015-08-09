#ifndef ROSCPP_MESSAGE_REFLECTION_NODE_HANDLE_H
#define ROSCPP_MESSAGE_REFLECTION_NODE_HANDLE_H

#include <string>
#include <ros/ros.h>
#include <roscpp_message_reflection/message_description_provider.h>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_impl.h>

namespace roscpp_message_reflection {

class Publisher {
public:
  Publisher() {}
  Publisher(ros::Publisher pub, MessageDescription::Ptr description)
    : pub_(pub), description_(description) {}

  operator bool() const {
    return pub_;
  }

  Message createMessage() {
    return Message(description_);
  }

  MessageDescription::Ptr getMessageType() {
    return description_;
  }

  void publish(Message& msg) {
    if(msg.getDescription() != description_) {
      ROS_WARN("Message description does not match publisher");
      return;
    }
    pub_.publish(msg);
  }

  void publish(const boost::shared_ptr<const Message>& msg) {
    if(msg->getDescription() != description_) {
      ROS_WARN("Message description does not match publisher");
      return;
    }
    pub_.publish(msg);
  }
private:
  ros::Publisher pub_;
  MessageDescription::Ptr description_;
};

class ServiceClient {
public:
  ServiceClient() {}
  ServiceClient(ros::ServiceClient client, ServiceDescription::Ptr description)
    : client_(client), description_(description) {}

  operator bool() const {
    return client_;
  }

  Message createRequestMessage() {
    return Message(description_->request);
  }

  Message createResponseMessage() {
    return Message(description_->response);
  }

  bool call(const Message& req, Message& res) {
    if(req.getDescription() != description_->request) {
      ROS_WARN("Message request description does not match service");
      return false;
    }
    if(res.getDescription() != description_->response) {
      ROS_WARN("Message response description does not match service");
      return false;
    }
    return client_.call(req, res, description_->md5sum);
  }
private:
  ros::ServiceClient client_;
  ServiceDescription::Ptr description_;
};

class Subscriber {
public:
  Subscriber() {}
  Subscriber(ros::Subscriber sub, MessageDescription::Ptr description)
    : sub_(sub), description_(description) {}

  operator bool() const {
    return sub_;
  }

private:
  ros::Subscriber sub_;
  MessageDescription::Ptr description_;
};

class ServiceServer {
public:
  ServiceServer() {}
  ServiceServer(ros::ServiceServer server, ServiceDescription::Ptr description)
    : server_(server), description_(description) {}

  operator bool() const {
    return server_;
  }

private:
  ros::ServiceServer server_;
  ServiceDescription::Ptr description_;
};

class NodeHandle {
public:
  NodeHandle(ros::NodeHandle& nh)
    : nh_(nh), description_provider_(MessageDescriptionProvider::Create(nh_)) {}

  Publisher advertise(const std::string& topic, const std::string& type) {
    MessageDescription::Ptr description = description_provider_->getMessageDescription(type);
    if(!description)
      return Publisher();

    ros::AdvertiseOptions options;
    options.has_header = false; // TODO handle this correctly
    options.topic = topic;
    options.datatype = type;
    options.md5sum = description->md5sum;
    options.message_definition = description->full_text;
    return Publisher(nh_.advertise(options), description);
  }

  class MessageCreator {
  public:
    MessageCreator(const MessageDescription::Ptr& description) : description_(description) {}
    boost::shared_ptr<Message> operator()() {
      return boost::make_shared<Message>(description_);
    }
  private:
    const MessageDescription::Ptr description_;
  };

  Subscriber subscribe(const std::string& topic, const std::string& type,
		       const boost::function< void(const boost::shared_ptr<const Message>&)> callback) {
    MessageDescription::Ptr description = description_provider_->getMessageDescription(type);
    if(!description)
      return Subscriber();

    ros::SubscribeOptions options;
    options.topic = topic;
    options.datatype = type;
    options.md5sum = description->md5sum;
    options.helper = ros::SubscriptionCallbackHelperPtr(
            new ros::SubscriptionCallbackHelperT<const boost::shared_ptr<Message const>&>(
	            callback, MessageCreator(description)));
    return Subscriber(nh_.subscribe(options), description);
  }

  ServiceServer advertiseService(const std::string& service, const std::string& type,
				 const boost::function<bool(const Message&, Message&)> callback) {
    ServiceDescription::Ptr description = description_provider_->getServiceDescription(type);
    if(!description)
      return ServiceServer();

    ros::AdvertiseServiceOptions options;
    options.service = service;
    options.datatype = type;
    options.req_datatype = description->request->name;
    options.res_datatype = description->response->name;
    options.md5sum = description->md5sum;
    options.helper = ros::ServiceCallbackHelperPtr(
            new ros::ServiceCallbackHelperT<ros::ServiceSpec<Message, Message> >(
	    callback, MessageCreator(description->request), MessageCreator(description->response)));
    return ServiceServer(nh_.advertiseService(options), description);
  }

  ServiceClient serviceClient(const std::string& service, const std::string& type) {
    ServiceDescription::Ptr description = description_provider_->getServiceDescription(type);
    if(!description)
      return ServiceClient();

    ros::ServiceClientOptions options;
    options.service = service;
    options.md5sum = description->md5sum;
    return ServiceClient(nh_.serviceClient(options), description);
  }

private:
  ros::NodeHandle nh_;
  MessageDescriptionProvider::Ptr description_provider_;
};

}

#endif
