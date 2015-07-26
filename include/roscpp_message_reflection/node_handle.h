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

  void publish(Message msg) {
    if(msg.getDescription() != description_) {
      ROS_WARN("Message description does not match publisher");
      return;
    }
    pub_.publish(msg);
  }
private:
  ros::Publisher pub_;
  MessageDescription::Ptr description_;
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
    MessageCreator(MessageDescription::Ptr& description) : description_(description) {}
    boost::shared_ptr<Message> operator()() {
      return boost::make_shared<Message>(description_);
    }
  private:
    MessageDescription::Ptr description_;
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

private:
  ros::NodeHandle nh_;
  MessageDescriptionProvider::Ptr description_provider_;
};

}

#endif
