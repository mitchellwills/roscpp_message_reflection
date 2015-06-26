#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H

#include <stdint.h>
#include <string>
#include <boost/any.hpp>

namespace roscpp_message_reflection {

class MessageValue {
public:
  template <typename T>
  static MessageValue Create() {
    MessageValue value;
    value.morph<T>();
    return value;
  }

  bool empty() { return value_.empty(); }
  template <typename T> void operator=(const T& other) {
    if(typeid(T) == value_.type()) {
      value_ = other;
    }
    else {
      std::stringstream ss;
      ss << "The assigned value must match the field c++ type exactly. Expected: ";
      ss << value_.type().name() << ", Got: " << typeid(T).name();
      throw MessageException(ss.str());
    }
  }
  template <typename OtherT> OtherT as() const {
    return boost::any_cast<OtherT>(value_);
  }

  template <typename T> void morph() {
    value_ = T();
  }

#define SIMPLE_DESERIALIZER(type_name)			\
  else if(value_.type() == typeid(type_name)) {		\
    type_name deserialized_value;			\
    stream.next(deserialized_value);			\
    value_ = deserialized_value;			\
  }
  template<typename Stream>
  void deserialize(Stream& stream) {
    if(false){}
    SIMPLE_DESERIALIZER(int8_t)
    SIMPLE_DESERIALIZER(uint8_t)
    SIMPLE_DESERIALIZER(int16_t)
    SIMPLE_DESERIALIZER(uint16_t)
    SIMPLE_DESERIALIZER(int32_t)
    SIMPLE_DESERIALIZER(uint32_t)
    SIMPLE_DESERIALIZER(int64_t)
    SIMPLE_DESERIALIZER(uint64_t)
    SIMPLE_DESERIALIZER(float)
    SIMPLE_DESERIALIZER(double)
    SIMPLE_DESERIALIZER(std::string)
    SIMPLE_DESERIALIZER(ros::Time)
    SIMPLE_DESERIALIZER(ros::Duration)
    else {
      std::stringstream ss;
      ss << "Failed to deserialize: " << value_.type().name();
      throw MessageException(ss.str());
    }
  }

#define SIMPLE_SERIALIZER(type_name)			\
  else if(value_.type() == typeid(type_name)) {		\
    stream.next(as<type_name>());			\
  }
  template<typename Stream>
  void serialize(Stream& stream) const {
    if(false){}
    SIMPLE_SERIALIZER(int8_t)
    SIMPLE_SERIALIZER(uint8_t)
    SIMPLE_SERIALIZER(int16_t)
    SIMPLE_SERIALIZER(uint16_t)
    SIMPLE_SERIALIZER(int32_t)
    SIMPLE_SERIALIZER(uint32_t)
    SIMPLE_SERIALIZER(int64_t)
    SIMPLE_SERIALIZER(uint64_t)
    SIMPLE_SERIALIZER(float)
    SIMPLE_SERIALIZER(double)
    SIMPLE_SERIALIZER(std::string)
    SIMPLE_SERIALIZER(ros::Time)
    SIMPLE_SERIALIZER(ros::Duration)
    else {
      std::stringstream ss;
      ss << "Failed to serialize: " << value_.type().name();
      throw MessageException(ss.str());
    }
  }

private:
  // TODO: replace with boost::variant ?
  boost::any value_;
};

}

#endif
