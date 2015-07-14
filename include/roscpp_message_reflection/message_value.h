#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H

#include <stdint.h>
#include <string>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/value_array.h>
#include <roscpp_message_reflection/message_array.h>
#include <roscpp_message_reflection/message_exception.h>
#include <roscpp_message_reflection/util.h>
#include <boost/variant.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/icl/type_traits/is_numeric.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/type_traits.hpp>
#include <boost/foreach.hpp>
#include <ros/time.h>
#include <ros/duration.h>

namespace roscpp_message_reflection {

typedef uint8_t RosBool;
typedef int8_t RosInt8;
typedef uint8_t RosUint8;
typedef int16_t RosInt16;
typedef uint16_t RosUint16;
typedef int32_t RosInt32;
typedef uint32_t RosUint32;
typedef int64_t RosInt64;
typedef uint64_t RosUint64;
typedef float RosFloat32;
typedef double RosFloat64;
typedef ros::Time RosTime;
typedef ros::Duration RosDuration;
typedef std::string RosString;

// Depricated ROS types
typedef int8_t RosByte;
typedef uint8_t RosChar;

class Message;

template <typename Stream>
class istream_next_visitor
  : public boost::static_visitor<>
{
public:
  istream_next_visitor(Stream& stream)
    : stream_(stream) {}
  template <typename T>
  void operator()(T& value)
  {
    stream_.next(value);
  }

  template <typename T>
  void operator()(ValueArray<T>& array)
  {
    uint32_t new_size;
    stream_.next(new_size);
    array.resize(new_size);
    for(size_t i = 0; i < array.size(); ++i) {
      stream_.next(array[i]);
    }
  }

  void operator()(MessageArray& array)
  {
    uint32_t new_size;
    stream_.next(new_size);
    array.resize(new_size);
    for(size_t i = 0; i < array.size(); ++i) {
      stream_.next(array[i]);
    }
  }
private:
  Stream& stream_;
};

template <typename Stream>
class ostream_next_visitor
  : public boost::static_visitor<>
{
public:
  ostream_next_visitor(Stream& stream)
    : stream_(stream) {}
  template <typename T>
  void operator()(const T& value)
  {
    stream_.next(value);
  }

  template <typename T>
  void operator()(const ValueArray<T>& array)
  {
    stream_.next((uint32_t)array.size());
    for(size_t i = 0; i < array.size(); ++i) {
      stream_.next(array[i]);
    }
  }

  void operator()(const MessageArray& array)
  {
    stream_.next((uint32_t)array.size());
    for(size_t i = 0; i < array.size(); ++i) {
      stream_.next(array[i]);
    }
  }
private:
  Stream& stream_;
};

template <typename TargetT, typename InputT>
typename boost::enable_if_c<!(boost::is_arithmetic<InputT>::value && boost::is_arithmetic<TargetT>::value)
  && !boost::is_same<typename boost::remove_const<TargetT>::type, typename boost::remove_const<InputT>::type>::value, TargetT>::type
convert(const InputT& value)
{
  std::stringstream ss;
  ss << "Cannot cast " << demangle_type<InputT>() << " to " << demangle_type<TargetT>();
  throw MessageException(ss.str());
}

template <typename TargetT, typename InputT>
typename boost::enable_if_c<boost::is_arithmetic<InputT>::value
&& boost::is_arithmetic<TargetT>::value
&& !boost::is_same<typename boost::remove_const<TargetT>::type, typename boost::remove_const<InputT>::type>::value, TargetT>::type
convert(const InputT& value)
{
  return boost::numeric_cast<TargetT, InputT>(value);
}

template <typename TargetT, typename InputT>
typename boost::enable_if_c<boost::is_same<typename boost::remove_const<TargetT>::type, typename boost::remove_const<InputT>::type>::value, TargetT>::type
convert(const InputT& value)
{
  return value;
}

template <typename InputT>
class assignment_visitor
  : public boost::static_visitor<>
{
public:
  assignment_visitor(const InputT& new_value)
    : new_value_(new_value) {}

  template <typename T>
  void operator()(T& value)
  {
    value = convert<T, InputT>(new_value_);
  }
private:
  const InputT& new_value_;
};

// need to specialize so that char* is converted to a string
// TODO need to figure out how to combine these better
template <std::size_t N>
class assignment_visitor<char[N]>
  : public boost::static_visitor<>
{
public:
  assignment_visitor(const char* new_value)
    : new_value_(new_value) {}

  template <typename T>
  void operator()(T& value)
  {
    value = convert<T, std::string>(std::string(new_value_));
  }
private:
  const char* new_value_;
};
template <>
class assignment_visitor<const char*>
  : public boost::static_visitor<>
{
public:
  assignment_visitor(const char* new_value)
    : new_value_(new_value) {}

  template <typename T>
  void operator()(T& value)
  {
    value = convert<T, std::string>(std::string(new_value_));
  }
private:
  const char* new_value_;
};
template <>
class assignment_visitor<char*>
  : public boost::static_visitor<>
{
public:
  assignment_visitor(char* new_value)
    : new_value_(new_value) {}

  template <typename T>
  void operator()(T& value)
  {
    value = convert<T, std::string>(std::string(new_value_));
  }
private:
  const char* new_value_;
};


template <typename TargetT>
class as_visitor
  : public boost::static_visitor<TargetT>
{
public:
  template <typename T>
  TargetT operator()(T& value)
  {
    return convert<TargetT, T>(value);
  }
};

class size_visitor
  : public boost::static_visitor<size_t>
{
public:
  template <typename T>
  size_t operator()(T& value) const
  {
    std::stringstream ss;
    ss << "Cannot get size of " << demangle_type<T>();
    throw MessageException(ss.str());
  }
  template <typename T>
  size_t operator()(const ValueArray<T>& array) const
  {
    return array.size();
  }
  size_t operator()(const MessageArray& array) const
  {
    return array.size();
  }
};

template <typename ValueT>
class get_array_index_visitor
  : public boost::static_visitor<ValueT&>
{
public:
  get_array_index_visitor(size_t index) : index_(index) {}

  template <typename T>
  ValueT& operator()(T& value) const
  {
    std::stringstream ss;
    ss << "Cannot get indexed value from non-array " << demangle_type<T>();
    throw MessageException(ss.str());
  }
  template <typename T>
  ValueT& operator()(ValueArray<T>& value) const
  {
    std::stringstream ss;
    ss << "Cannot get indexed value from mismatched array of type: " << demangle_type<T>()
       << ", looking for array of type: " << demangle_type<ValueT>();
    throw MessageException(ss.str());
  }
  ValueT& operator()(ValueArray<ValueT>& array) const
  {
    return array[index_];
  }

private:
  size_t index_;
};

// TODO figure out how to inherit some of these from the general version
template<>
class get_array_index_visitor<Message>
  : public boost::static_visitor<Message&>
{
public:
  get_array_index_visitor(size_t index) : index_(index) {}

  template <typename T>
  Message& operator()(T& value) const
  {
    std::stringstream ss;
    ss << "Cannot get indexed value from non-array " << demangle_type<T>();
    throw MessageException(ss.str());
  }
  template <typename T>
  Message& operator()(ValueArray<T>& value) const
  {
    std::stringstream ss;
    ss << "Cannot get indexed value from mismatched array of type: " << demangle_type<T>()
       << ", looking for array of type: " << demangle_type<Message>();
    throw MessageException(ss.str());
  }

  Message& operator()(MessageArray& array) const
  {
    return array[index_];
  }
private:
  size_t index_;
};

class resize_visitor
  : public boost::static_visitor<>
{
public:
  resize_visitor(size_t new_size) : new_size_(new_size) {}

  template <typename T>
  void operator()(T& value) const
  {
    std::stringstream ss;
    ss << "Cannot resize a non-array " << demangle_type<T>();
    throw MessageException(ss.str());
  }
  template <typename T>
  void operator()(ValueArray<T>& array) const
  {
    return array.resize(new_size_);
  }
  void operator()(MessageArray& array) const
  {
    return array.resize(new_size_);
  }

private:
  size_t new_size_;
};

class MessageValue {
private:
  MessageValue() {}
public:

  template <typename T>
  static MessageValue Create(const T& initial_value = T()) {
    MessageValue value;
    value.morph<T>(initial_value);
    return value;
  }

  static MessageValue Create(const MessageDescription::Ptr& value_type) {
    MessageValue value;
    value.morph<Message>(Message(value_type));
    return value;
  }

  template <typename ValueT>
  static MessageValue CreateVariableLengthArray(size_t size = 0) {
    MessageValue value;
    value.morph<ValueArray<ValueT> >(ValueArray<ValueT>(false, size));
    return value;
  }

  static MessageValue CreateVariableLengthArray(const MessageDescription::Ptr& value_type, size_t size = 0) {
    MessageValue value;
    value.morph<MessageArray>(MessageArray(value_type, false, size));
    return value;
  }

  template <typename ValueT>
  static MessageValue CreateFixedLengthArray(size_t size) {
    MessageValue value;
    value.morph<ValueArray<ValueT> >(ValueArray<ValueT>(true, size));
    return value;
  }

  static MessageValue CreateFixedLengthArray(const MessageDescription::Ptr& value_type, size_t size) {
    MessageValue value;
    value.morph<MessageArray>(MessageArray(value_type, true, size));
    return value;
  }

  template <typename T> void operator=(const T& other) {
    assignment_visitor<T> visitor(other);
    boost::apply_visitor(visitor, value_);
  }

  // This matches the vector of types below
  enum ValueType {
    int8,
    uint8,
    int16,
    uint16,
    int32,
    uint32,
    int64,
    uint64,
    float32,
    float64,
    string,
    time,
    duration,
    message,
    ValueTypeCount
  };

  ValueType valueType() {
    return (ValueType)(value_.which() % ValueTypeCount);
  }

  bool isArray() {
    return value_.which() >= ValueTypeCount;
  }

  bool operator==(const MessageValue& other) const {
    return value_ == other.value_;
  }
  bool operator!=(const MessageValue& other) const {
    return !operator==(other);
  }


  template <typename OtherT> OtherT as() const {
    as_visitor<OtherT> visitor;
    return boost::apply_visitor(visitor, value_);
  }

  MessageValue& operator[](const std::string& name) {
    return get<Message>()[name];
  }

  template <typename T> T& get(){
    return boost::get<T>(value_);
  }

  template <typename T> T& get(size_t index){
    get_array_index_visitor<T> visitor(index);
    return boost::apply_visitor(visitor, value_);
  }

  size_t size() const {
     return boost::apply_visitor(size_visitor(), value_);
  }

  void resize(size_t size) {
    resize_visitor visitor(size);
    boost::apply_visitor(visitor, value_);
  }

  template <typename T>
  void morph(const T& new_value = T()) {
    value_ = new_value;
  }


  template<typename Stream>
  void deserialize(Stream& stream) {
    istream_next_visitor<Stream> visitor(stream);
    boost::apply_visitor(visitor, value_);
  }

  template<typename Stream>
  void serialize(Stream& stream) const {
    ostream_next_visitor<Stream> visitor(stream);
    boost::apply_visitor(visitor, value_);
  }

private:
  typedef boost::mpl::vector<
  int8_t,
  uint8_t,
  int16_t,
  uint16_t,
  int32_t,
  uint32_t,
  int64_t,
  uint64_t,
  float,
  double,
  std::string,
  ros::Time,
  ros::Duration,
  Message,
  ValueArray<int8_t>,
  ValueArray<uint8_t>,
  ValueArray<int16_t>,
  ValueArray<uint16_t>,
  ValueArray<int32_t>,
  ValueArray<uint32_t>,
  ValueArray<int64_t>,
  ValueArray<uint64_t>,
  ValueArray<float>,
  ValueArray<double>,
  ValueArray<std::string>,
  ValueArray<ros::Time>,
  ValueArray<ros::Duration>,
  MessageArray
  > value_type_vec;
  typedef boost::make_variant_over<value_type_vec>::type value_type;
  value_type value_;
};

}

#endif
