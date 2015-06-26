#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_VALUE_H

#include <stdint.h>
#include <string>
#include <boost/variant.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/icl/type_traits/is_numeric.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/type_traits.hpp>
#include <ros/time.h>
#include <ros/duration.h>

namespace roscpp_message_reflection {

template <typename Stream>
class stream_next_visitor
  : public boost::static_visitor<>
{
public:
  stream_next_visitor(Stream& stream)
    : stream_(stream) {}
  template <typename T>
  void operator()(T& value)
  {
    stream_.next(value);
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
  ss << "Cannot cast " << typeid(InputT).name() << " to " << typeid(TargetT).name();
  throw std::runtime_error(ss.str());
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
class get_visitor
  : public boost::static_visitor<TargetT>
{
public:
  template <typename T>
  TargetT operator()(T& value)
  {
    return convert<TargetT, T>(value);
  }
};

class MessageValue {
public:
  template <typename T>
  static MessageValue Create() {
    MessageValue value;
    value.morph<T>();
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
    get_visitor<OtherT> visitor;
    return boost::apply_visitor(visitor, value_);
  }

  template <typename T> void morph() {
    value_ = T();
  }

  template<typename Stream>
  void deserialize(Stream& stream) {
    stream_next_visitor<Stream> visitor(stream);
    boost::apply_visitor(visitor, value_);
  }

  template<typename Stream>
  void serialize(Stream& stream) const {
    stream_next_visitor<Stream> visitor(stream);
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
  ros::Duration
  > value_type_vec;
  typedef boost::make_variant_over<value_type_vec>::type value_type;
  value_type value_;
};

}

#endif
