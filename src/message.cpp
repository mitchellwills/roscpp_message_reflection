#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_value.h>

namespace roscpp_message_reflection {

MessageValue& Message::operator[](const std::string& name) {
  std::map<std::string, MessageValue>::iterator itr = fields_.find(name);
  if(itr == fields_.end()) {
    throw MessageException("Message: " + description_->name + " does not contain a field: " + name);
  }
  return itr->second;
}

#define VALUE_MORPH(label, type_name)					\
  else if(field.value_type() == label) {				\
    fields_[field.name()] = MessageValue::Create<type_name>();		\
  }
#define VALUE_MORPH_INT(int_type)		\
  VALUE_MORPH(#int_type, int_type##_t)

void Message::morph(MessageDescription::Ptr description) {
  if(description_ == description)
    return;

  description_ = description;
  fields_.clear();
  BOOST_FOREACH(const FieldDescription& field, description_->fields) {
    if(field.type() == FieldDescription::Value) {
      if(false) {}
      VALUE_MORPH("bool", uint8_t)
      VALUE_MORPH("byte", int8_t)
      VALUE_MORPH("char", uint8_t)
      VALUE_MORPH_INT(int8)
      VALUE_MORPH_INT(uint8)
      VALUE_MORPH_INT(int16)
      VALUE_MORPH_INT(uint16)
      VALUE_MORPH_INT(int32)
      VALUE_MORPH_INT(uint32)
      VALUE_MORPH_INT(int64)
      VALUE_MORPH_INT(uint64)
      VALUE_MORPH("float32", float)
      VALUE_MORPH("float64", double)
      VALUE_MORPH("string", std::string)
      VALUE_MORPH("time", ros::Time)
      VALUE_MORPH("duration", ros::Duration)
      else {
	throw MessageException("Unsupported field value type: " + field.value_type());
      }
    }
    else {
      throw MessageException("MessageValue only supports value fields");
    }
  }
}

}
