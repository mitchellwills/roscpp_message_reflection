#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_IMPL_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_IMPL_H

#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_description.h>
#include <roscpp_message_reflection/message_value.h>
#include <boost/foreach.hpp>

namespace roscpp_message_reflection {

struct Message::FieldEntry {
  FieldEntry(const std::string& name, const MessageValue& value);
  std::string name;
  MessageValue value;
};

template<typename Stream>
void Message::read(Stream& stream) {
  BOOST_FOREACH(FieldEntry& entry, fields_) {
    entry.value.deserialize(stream);
  }
}

template<typename Stream>
void Message::write(Stream& stream) const {
  BOOST_FOREACH(const FieldEntry& entry, fields_) {
    entry.value.serialize(stream);
  }
}

}

namespace ros {
namespace serialization {

template<typename Stream>
void Serializer<roscpp_message_reflection::Message>::write(Stream& stream, const roscpp_message_reflection::Message& m) {
  m.write(stream);
}


template<typename Stream>
void Serializer<roscpp_message_reflection::Message>::read(Stream& stream, roscpp_message_reflection::Message& m)
{
  m.read(stream);
}

}
}

#endif
