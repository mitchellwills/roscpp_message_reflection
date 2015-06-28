#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_impl.h>
#include <roscpp_message_reflection/message_value.h>
#include <boost/foreach.hpp>

namespace roscpp_message_reflection {

Message::FieldEntry::FieldEntry(const std::string& name, const MessageValue& value)
  : name(name), value(value) {}

Message::Message() {}
Message::Message(const Message& other) {
  description_ = other.description_;
  fields_ = other.fields_;
}
Message& Message::operator=(const Message& other) {
  description_ = other.description_;
  fields_ = other.fields_;
}
Message::~Message() {}

MessageValue& Message::operator[](const std::string& name) {
  assertValid();
  BOOST_FOREACH(FieldEntry& entry, fields_) {
    if(entry.name == name) {
      return entry.value;
    }
  }
  throw MessageException("Message: " + description_->name + " does not contain a field: " + name);
}


class value_field_construction_visitor
{
public:
  typedef void result_type;
  value_field_construction_visitor(std::vector<Message::FieldEntry>& fields, const std::string& field_name)
    : fields_(fields), field_name_(field_name) {}
  template <typename T>
  result_type operator()()
  {
    fields_.push_back(Message::FieldEntry(field_name_, MessageValue::Create<T>()));
  }
  result_type operator()(MessageDescription::Ptr message_description)
  {
    Message message;
    message.morph(message_description);
    fields_.push_back(Message::FieldEntry(field_name_, MessageValue::Create(message)));
  }
private:
  const std::string field_name_;
  std::vector<Message::FieldEntry>& fields_;
};

class variable_length_array_field_construction_visitor
{
public:
  typedef void result_type;
  variable_length_array_field_construction_visitor(std::vector<Message::FieldEntry>& fields, const std::string& field_name)
    : fields_(fields), field_name_(field_name) {}
  template <typename T>
  result_type operator()()
  {
    fields_.push_back(Message::FieldEntry(field_name_, MessageValue::CreateArray<T>()));
  }
  result_type operator()(MessageDescription::Ptr message_description)
  {
    fields_.push_back(Message::FieldEntry(field_name_, MessageValue::Create(MessageArray(message_description))));
  }
private:
  const std::string field_name_;
  std::vector<Message::FieldEntry>& fields_;
};

void Message::morph(MessageDescription::Ptr description) {
  if(description_ == description)
    return;

  description_ = description;
  fields_.clear();
  BOOST_FOREACH(const FieldDescription& field, description_->fields) {
    if(field.type() == FieldDescription::Value) {
      value_field_construction_visitor visitor(fields_, field.name());
      visit_field_type(description, visitor, field.value_type());
    }
    else if(field.type() == FieldDescription::VariableLengthArray) {
      variable_length_array_field_construction_visitor visitor(fields_, field.name());
      visit_field_type(description, visitor, field.value_type());
    }
    else {
      throw MessageException("MessageValue only supports value or variable length array fields");
    }
  }
}

}
