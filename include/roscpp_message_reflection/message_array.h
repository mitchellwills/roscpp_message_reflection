#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_ARRAY_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_ARRAY_H

#include <string>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_description.h>

namespace roscpp_message_reflection {

class MessageArray {
public:
  MessageArray(const MessageDescription::Ptr& value_type)
    : value_type_(value_type) {}
  Message& operator[](size_t index) {
    return array_[index];
  }
  const Message& operator[](size_t index) const {
    return array_[index];
  }

  void resize(size_t new_size) {
    size_t old_size = array_.size();
    array_.resize(new_size);
    for(size_t i = old_size; i < new_size; ++i) {
      array_[i].morph(value_type_);
    }
  }

  size_t size() const {
    return array_.size();
  }
private:
  std::vector<Message> array_;
  MessageDescription::Ptr value_type_;
};

}

#endif
