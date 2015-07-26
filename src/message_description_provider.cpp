#include <roscpp_message_reflection/message_description_provider.h>
#include <boost/foreach.hpp>
#include <roscpp_message_reflection/GetServiceInfo.h>

namespace roscpp_message_reflection {

MessageDescriptionProvider* MessageDescriptionProvider::Create(ros::NodeHandle& nh) {
  return new MessageReflectionServerMessageDescriptionProvider(nh);
}

MessageDescription::Ptr MessageDescriptionProvider::getMessageDescription(const std::string& type) {
  MessageCacheType::iterator itr = message_cache_.find(type);
  if(itr != message_cache_.end()) {
    return itr->second;
  }
  else {
    fillMessageCache(type);
    itr = message_cache_.find(type);
    if(itr != message_cache_.end()) {
      return itr->second;
    }
    else {
      return MessageDescription::Ptr();
    }
  }
}

ServiceDescription::Ptr MessageDescriptionProvider::getServiceDescription(const std::string& type) {
  ServiceCacheType::iterator itr = service_cache_.find(type);
  if(itr != service_cache_.end()) {
    return itr->second;
  }
  else {
    fillServiceCache(type);
    itr = service_cache_.find(type);
    if(itr != service_cache_.end()) {
      return itr->second;
    }
    else {
      return ServiceDescription::Ptr();
    }
  }
}

MessageReflectionServerMessageDescriptionProvider::MessageReflectionServerMessageDescriptionProvider(ros::NodeHandle& nh)
  : nh_(nh) {
  message_description_service_ = nh_.serviceClient<roscpp_message_reflection::GetMessageInfo>("get_message_info");
  service_description_service_ = nh_.serviceClient<roscpp_message_reflection::GetServiceInfo>("get_service_info");
  message_description_service_.waitForExistence(ros::Duration(1));
  service_description_service_.waitForExistence(ros::Duration(1));
}

void MessageReflectionServerMessageDescriptionProvider::fillMessageCache(const std::vector<MessageInfo>& infos) {
  BOOST_REVERSE_FOREACH(const MessageInfo& info, infos) {
    std::vector<FieldDescription> fields;
    std::map<std::string, MessageDescription::Ptr> child_messages;
    BOOST_FOREACH(const MessageFieldInfo& field_info, info.fields) {
      FieldDescription field = FieldDescription::CreateFromFullType(field_info.name, field_info.type);
      fields.push_back(field);
      MessageDescription::Ptr field_value_message = getMessageDescription(field.value_type());
      if(field_value_message) {
	child_messages[field_value_message->name] = field_value_message;
      }
    }
    message_cache_[info.type] = MessageDescription::Ptr(new MessageDescription(info.type, info.md5sum, info.definition, fields, child_messages));
  }
}

void MessageReflectionServerMessageDescriptionProvider::fillMessageCache(const std::string& type) {
  GetMessageInfo::Request request;
  request.type = type;
  GetMessageInfo::Response response;
  if(message_description_service_.call(request, response)) {
    fillMessageCache(response.infos);
  }
  else {
    ROS_ERROR("Service call failed");
  }
}

void MessageReflectionServerMessageDescriptionProvider::fillServiceCache(const std::string& type) {
  GetServiceInfo::Request request;
  request.type = type;
  GetServiceInfo::Response response;
  if(service_description_service_.call(request, response)) {
    fillMessageCache(response.message_infos);
    service_cache_[response.service.type] = ServiceDescription::Ptr(new ServiceDescription(response.service.type, response.service.md5sum, getMessageDescription(response.service.req_type), getMessageDescription(response.service.res_type)));
  }
  else {
    ROS_ERROR("Service call failed");
  }
}

}
