#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_DESCRIPTION_PROVIDER_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_DESCRIPTION_PROVIDER_H

#include <map>
#include <string>
#include <roscpp_message_reflection/message_description.h>
#include <ros/ros.h>

namespace roscpp_message_reflection {

class MessageDescriptionProvider {
public:
  typedef boost::shared_ptr<MessageDescriptionProvider> Ptr;

  static MessageDescriptionProvider* Create(ros::NodeHandle& nh);

  MessageDescription::Ptr getDescription(const std::string& type);
protected:
  virtual void fillCache(const std::string& type) = 0;

  typedef std::map<std::string, MessageDescription::Ptr> CacheType;
  CacheType cache_;
};

class MessageReflectionServerMessageDescriptionProvider : public MessageDescriptionProvider {
public:
  MessageReflectionServerMessageDescriptionProvider(ros::NodeHandle& nh);
protected:
  virtual void fillCache(const std::string& type);
private:
  ros::NodeHandle nh_;
  ros::ServiceClient description_service_;
};

}

#endif
