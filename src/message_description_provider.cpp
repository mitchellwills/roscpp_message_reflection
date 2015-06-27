#include <roscpp_message_reflection/message_description_provider.h>
#include <roscpp_message_reflection/GetMessageInfo.h>
#include <boost/foreach.hpp>

namespace roscpp_message_reflection {

MessageDescriptionProvider* MessageDescriptionProvider::Create(ros::NodeHandle& nh) {
  return new MessageReflectionServerMessageDescriptionProvider(nh);
}

MessageDescription::Ptr MessageDescriptionProvider::getDescription(const std::string& type) {
  CacheType::iterator itr = cache_.find(type);
  if(itr != cache_.end()) {
    return itr->second;
  }
  else {
    fillCache(type);
    itr = cache_.find(type);
    if(itr != cache_.end()) {
      return itr->second;
    }
    else {
      return MessageDescription::Ptr();
    }
  }
}

MessageReflectionServerMessageDescriptionProvider::MessageReflectionServerMessageDescriptionProvider(ros::NodeHandle& nh)
  : nh_(nh) {
  description_service_ = nh_.serviceClient<roscpp_message_reflection::GetMessageInfo>("get_message_info");
  description_service_.waitForExistence(ros::Duration(1));
}

void MessageReflectionServerMessageDescriptionProvider::fillCache(const std::string& type) {
  GetMessageInfo::Request request;
  request.type = type;
  GetMessageInfo::Response response;
  if(description_service_.call(request, response)) {
    BOOST_REVERSE_FOREACH(const MessageInfo& info, response.infos) {
      std::vector<FieldDescription> fields;
      std::map<std::string, MessageDescription::Ptr> child_messages;
      BOOST_FOREACH(const MessageFieldInfo& field_info, info.fields) {
	FieldDescription field = FieldDescription::CreateFromFullType(field_info.name, field_info.type);
	fields.push_back(field);
	MessageDescription::Ptr field_value_message = getDescription(field.value_type());
	if(field_value_message) {
	  child_messages[field_value_message->name] = field_value_message;
	}
      }
      cache_[info.type] = MessageDescription::Ptr(new MessageDescription(info.type, info.md5sum, info.definition, fields, child_messages));
    }
  }
  else {
    ROS_ERROR("Service call failed");
  }
}



}
