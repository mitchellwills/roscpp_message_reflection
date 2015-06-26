#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H

#include <boost/any.hpp>

namespace roscpp_message_reflection {

class MessageValue {
public:
  bool empty() { return value_.empty(); }
  template <typename T> void operator=(const T& other) {
    value_ = other;
  }
  template <typename OtherT> OtherT as() {
    return boost::any_cast<OtherT>(value_);
  }
private:
  template <typename T> void morph() {
    value_ = T();
  }

  // TODO: replace with boost::variant ?
  boost::any value_;
};

}

#endif
