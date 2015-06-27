#include <roscpp_message_reflection/message_value.h>
#include <roscpp_message_reflection/message.h>

namespace roscpp_message_reflection {

MessageValue& MessageValue::operator[](const std::string& name) {
  return as<MessageT>()->operator[](name);
}

MessageValue MessageValue::Create(Message& contents) {
  MessageValue value;
  value.value_ = MessageT(new Message(contents));
  return value;
}

}
