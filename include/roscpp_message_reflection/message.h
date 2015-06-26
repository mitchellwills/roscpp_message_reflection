#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_H

#include <string>
#include <map>
#include <roscpp_message_reflection/message_description.h>
#include <roscpp_message_reflection/message_value.h>
#include <boost/foreach.hpp>

namespace roscpp_message_reflection {

class Message {
public:
  MessageValue& operator[](const std::string& name);

  template<typename Stream>
  void read(Stream& stream) {
    for(std::map<std::string, MessageValue>::iterator field_itr = fields_.begin();
	field_itr != fields_.end(); ++field_itr) {
      field_itr->second.deserialize(stream);
    }
  }

  template<typename Stream>
  void write(Stream& stream) const {
    for(std::map<std::string, MessageValue>::const_iterator field_itr = fields_.begin();
	field_itr != fields_.end(); ++field_itr) {
      field_itr->second.serialize(stream);
    }
  }

  void morph(MessageDescription::Ptr description);

private:
  MessageDescription::Ptr description_;
  std::map<std::string, MessageValue> fields_; // std::map is ordered
};

}

namespace ros {
namespace serialization {

  template<>
  struct Serializer<roscpp_message_reflection::Message>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const roscpp_message_reflection::Message& m) {
      m.write(stream);
    }

    template<typename Stream>
    inline static void read(Stream& stream, roscpp_message_reflection::Message& m)
    {
      m.read(stream);
    }

    inline static uint32_t serializedLength(const roscpp_message_reflection::Message& m) {
      LStream stream;
      write(stream, m);
      return stream.getLength();
      // TODO make this recursive instead of writing the message out
      //return m.serializedLength();
    }
  };
}
}

#endif
