#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_value.h>

namespace roscpp_message_reflection {

MessageValue& Message::operator[](const std::string& name) {
  std::map<std::string, MessageValue>::iterator itr = values.find(name);
  if(itr != values.end()) {
    throw MessageException("Message: " + description.name + " does not contain a field: " + name);
  }
  return values[name];
}

void morph(MessageDescription::Ptr& message_description) {
  
}

}
