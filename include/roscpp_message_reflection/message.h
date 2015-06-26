#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_H

#include <string>
#include <map>
#include <roscpp_message_reflection/message_description.h>
#include <roscpp_message_reflection/message_value.h>

namespace roscpp_message_reflection {

class Message {
public:
  MessageValue& operator[](const std::string& name);

private:
  void morph(const MessageDescription& message_description);

  MessageDescription description;
  std::map<std::string, MessageValue> values;
};

}

#endif
