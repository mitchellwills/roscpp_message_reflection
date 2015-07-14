#ifndef ROSCPP_MESSAGE_REFLECTION_VALUE_ARRAY_H
#define ROSCPP_MESSAGE_REFLECTION_VALUE_ARRAY_H

#include <string>
#include <vector>
#include "roscpp_message_reflection/message_exception.h"

namespace roscpp_message_reflection {

template <typename T>
class ValueArray {
public:
  ValueArray(bool fixed_size, size_t size)
    : array_(size), fixed_size_(fixed_size) {}

  T& operator[](size_t index) {
    return array_[index];
  }
  const T& operator[](size_t index) const {
    return array_[index];
  }

  bool operator==(const ValueArray& other) const {
    return array_ == other.array_;
  }
  bool operator!=(const ValueArray& other) const {
    return !operator==(other);
  }

  void resize(size_t new_size) {
    if(fixed_size_) {
      throw MessageException("Cannot resize a fixed size array");
    }
    else {
      array_.resize(new_size);
    }
  }

  size_t size() const {
    return array_.size();
  }
private:
  std::vector<T> array_;
  bool fixed_size_;
};

}

#endif
