#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_value.h>
#include <boost/foreach.hpp>

namespace roscpp_message_reflection {

Message::FieldEntry::FieldEntry(const std::string& name, const MessageValue& value)
  : name(name), value(value) {}

Message::Message() {}
Message::~Message() {}

MessageValue& Message::operator[](const std::string& name) {
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
    else {
      throw MessageException("MessageValue only supports value fields");
    }
  }
}

}
