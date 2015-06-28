#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H

#include <stdint.h>
#include <string>
#include <roscpp_message_reflection/message.h>
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
  void operator()(std::vector<T>& array)
  {
    stream_.next(array);
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
  void operator()(const std::vector<T>& array)
  {
    stream_.next(array);
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
  size_t operator()(const std::vector<T>& array) const
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
  ValueT& operator()(std::vector<T>& value) const
  {
    std::stringstream ss;
    ss << "Cannot get indexed value from mismatched array of type: " << demangle_type<T>()
       << ", looking for array of type: " << demangle_type<ValueT>();
    throw MessageException(ss.str());
  }
  ValueT& operator()(std::vector<ValueT>& array) const
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
  void operator()(std::vector<T>& array) const
  {
    return array.resize(new_size_);
  }

private:
  size_t new_size_;
};


class MessageValue {
public:

  template <typename T>
  static MessageValue Create() {
    MessageValue value;
    value.morph<T>();
    return value;
  }
  template <typename T>
  static MessageValue CreateArray() {
    MessageValue value;
    value.morph<std::vector<T> >();
    return value;
  }
  template <typename T>
  static MessageValue Create(T contents) {
    MessageValue value;
    value.value_ = contents;
    return value;
  }

  template <typename T> void operator=(const T& other) {
    assignment_visitor<T> visitor(other);
    boost::apply_visitor(visitor, value_);
  }
  // need to specialize so that char* is converted to a string
  void operator=(const char* other) {
    operator=<std::string>(other);
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

  template <typename T> void morph() {
    value_ = T();
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
  std::vector<int8_t>,
  std::vector<uint8_t>,
  std::vector<int16_t>,
  std::vector<uint16_t>,
  std::vector<int32_t>,
  std::vector<uint32_t>,
  std::vector<int64_t>,
  std::vector<uint64_t>,
  std::vector<float>,
  std::vector<double>,
  std::vector<std::string>,
  std::vector<ros::Time>,
  std::vector<ros::Duration>,
  std::vector<Message>
  > value_type_vec;
  typedef boost::make_variant_over<value_type_vec>::type value_type;
  value_type value_;
};

}

#endif
