#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_DESCRIPTION_PROVIDER_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_DESCRIPTION_PROVIDER_H

#include <map>
#include <string>
#include <roscpp_message_reflection/message_description.h>
#include <roscpp_message_reflection/service_description.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <roscpp_message_reflection/GetMessageInfo.h>

namespace roscpp_message_reflection {

class MessageDescriptionProvider {
public:
  typedef boost::shared_ptr<MessageDescriptionProvider> Ptr;

  static MessageDescriptionProvider* Create(ros::NodeHandle& nh);

  MessageDescription::Ptr getMessageDescription(const std::string& type);
  ServiceDescription::Ptr getServiceDescription(const std::string& type);
protected:
  virtual void fillMessageCache(const std::string& type) = 0;
  virtual void fillServiceCache(const std::string& type) = 0;

  typedef std::map<std::string, MessageDescription::Ptr> MessageCacheType;
  typedef std::map<std::string, ServiceDescription::Ptr> ServiceCacheType;
  MessageCacheType message_cache_;
  ServiceCacheType service_cache_;
};

class MessageReflectionServerMessageDescriptionProvider : public MessageDescriptionProvider {
public:
  MessageReflectionServerMessageDescriptionProvider(ros::NodeHandle& nh);
protected:
  void fillMessageCache(const std::vector<MessageInfo>& infos);
  virtual void fillMessageCache(const std::string& type);
  virtual void fillServiceCache(const std::string& type);
private:
  ros::NodeHandle nh_;
  ros::ServiceClient message_description_service_;
  ros::ServiceClient service_description_service_;
};

}

#endif
