#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_ARRAY_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_ARRAY_H

#include <string>
#include <vector>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_description.h>

namespace roscpp_message_reflection {

class MessageArray {
public:
  MessageArray(const MessageDescription::Ptr& value_type, bool fixed_size, size_t size)
    : value_type_(value_type), array_(size, Message(value_type_)), fixed_size_(fixed_size) {}

  Message& operator[](size_t index) {
    return array_[index];
  }
  const Message& operator[](size_t index) const {
    return array_[index];
  }

  bool operator==(const MessageArray& other) const {
    if(value_type_ != other.value_type_)
      return false;
    return array_ == other.array_;
  }
  bool operator!=(const MessageArray& other) const {
    return !operator==(other);
  }

  void resize(size_t new_size) {
    if(fixed_size_) {
      throw MessageException("Cannot resize a fixed size array");
    }
    else {
      Message value_template(value_type_);
      array_.resize(new_size, value_template);
    }
  }

  size_t size() const {
    return array_.size();
  }
private:
  MessageDescription::Ptr value_type_;
  std::vector<Message> array_;
  bool fixed_size_;
};

}

#endif
