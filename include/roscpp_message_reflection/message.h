#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_H

#include <string>
#include <map>
#include <roscpp_message_reflection/message_description.h>
#include <roscpp_message_reflection/message_value.h>

namespace roscpp_message_reflection {

class Message {
public:
  Message();
  ~Message();
  MessageValue& operator[](const std::string& name);

  template<typename Stream>
  void read(Stream& stream);

  template<typename Stream>
  void write(Stream& stream) const;

  void morph(MessageDescription::Ptr description);

private:
  struct FieldEntry {
    FieldEntry(const std::string& name, const MessageValue& value);
    std::string name;
    MessageValue value;
  };

  MessageDescription::Ptr description_;
  std::vector<FieldEntry> fields_;
};

}

namespace ros {
namespace serialization {

  template<>
  struct Serializer<roscpp_message_reflection::Message>
  {
    template<typename Stream>
    static void write(Stream& stream, const roscpp_message_reflection::Message& m);

    template<typename Stream>
    static void read(Stream& stream, roscpp_message_reflection::Message& m);

    static uint32_t serializedLength(const roscpp_message_reflection::Message& m);
  };
}
}

#endif
