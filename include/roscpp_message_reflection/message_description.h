#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_DESCRIPTION_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_DESCRIPTION_H

#include <stdint.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>
#include <roscpp_message_reflection/message_exception.h>
#include <ros/ros.h>

namespace roscpp_message_reflection {


class FieldDescription {
public:
  enum Type {
    Value, FixedLengthArray, VariableLengthArray
  };
  typedef uint32_t array_length_t;

  static FieldDescription CreateValue(const std::string& name, const std::string& type) {
    return FieldDescription(name, type, type, Value, 0);
  }

  static FieldDescription CreateVariableLengthArray(const std::string& name, const std::string& value_type) {
    std::stringstream full_type;
    full_type << value_type << "[]";
    return FieldDescription(name, full_type.str(), value_type, VariableLengthArray, 0);
  }

  static FieldDescription CreateFixedLengthArray(const std::string& name, const std::string& value_type, array_length_t array_length) {
    std::stringstream full_type;
    full_type << value_type << "[" << array_length << "]";
    return FieldDescription(name, full_type.str(), value_type, FixedLengthArray, array_length);
  }

  static FieldDescription CreateFromFullType(const std::string& name, const std::string& full_type) {
    static boost::regex field_type_regex("([^\\[]+)(\\[(\\d+)?\\])?");
    boost::smatch match;
    if(boost::regex_match(full_type, match, field_type_regex) && match.size() == 4) {
      if(match[2].length() > 0) { // is array
	std::string value_type = match[1];
	if(match[3].length() > 0) { // is fixed length
	  array_length_t length = boost::lexical_cast<array_length_t>(match[3]);
	  return CreateFixedLengthArray(name, value_type, length);
	}
	else {
	  return CreateVariableLengthArray(name, value_type);
	}
      }
      else {
	return CreateValue(name, full_type);
      }
    }
    else {
      throw MessageException("Failed to parse message field type");
    }
  }

  FieldDescription(const std::string& name, const std::string& full_type,
		   const std::string& value_type, Type type,
		   array_length_t array_length)
    : name_(name), full_type_(full_type), value_type_(value_type), type_(type),
      array_length_(array_length) {}

  std::string name() const { return name_; }
  std::string full_type() const {return full_type_; }
  std::string value_type() const { return value_type_; }
  Type type() const { return type_; }
  array_length_t array_length() const { return array_length_; }


private:
  std::string name_;
  std::string full_type_;
  std::string value_type_;
  Type type_;
  array_length_t array_length_;
};

struct MessageDescription {
  typedef boost::shared_ptr<MessageDescription> Ptr;

  MessageDescription(const std::string& name, const std::string& md5sum,
		     const std::string& full_text, std::vector<FieldDescription> fields,
		     std::map<std::string, MessageDescription::Ptr> child_messages)
    : name(name), md5sum(md5sum), full_text(full_text), fields(fields), child_messages(child_messages) {}

  const std::string name;
  const std::string md5sum;
  const std::string full_text;

  const std::vector<FieldDescription> fields;

  std::map<std::string, MessageDescription::Ptr> child_messages;
};


#define VISIT_TYPE(type_name, native_type)		\
  else if(field_type == type_name) {			\
    return visitor.template operator()<native_type>();	\
  }

#define VISIT_TYPE_INT(int_size)			\
  VISIT_TYPE("int"#int_size, int##int_size##_t)		\
  VISIT_TYPE("uint"#int_size, uint##int_size##_t)

template<typename Visitor>
typename Visitor::result_type visit_field_type(MessageDescription::Ptr& description, Visitor& visitor, const std::string& field_type) {
  if(false) {}
  VISIT_TYPE("bool", uint8_t)
  VISIT_TYPE("byte", int8_t)
  VISIT_TYPE("char", uint8_t)
  VISIT_TYPE_INT(8)
  VISIT_TYPE_INT(16)
  VISIT_TYPE_INT(32)
  VISIT_TYPE_INT(64)
  VISIT_TYPE("float32", float)
  VISIT_TYPE("float64", double)
  VISIT_TYPE("string", std::string)
  VISIT_TYPE("time", ros::Time)
  VISIT_TYPE("duration", ros::Duration)
  else {
    std::map<std::string, MessageDescription::Ptr>::iterator itr = description->child_messages.find(field_type);
    if(itr != description->child_messages.end()) {
      visitor(itr->second);
    }
    else {
      throw MessageException("Unsupported field value type: " + field_type);
    }
  }
}

}

#endif
