#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_H

#include <string>
#include <map>
#include <roscpp_message_reflection/message_description.h>

namespace roscpp_message_reflection {

class MessageValue;

class Message {
public:
  struct FieldEntry;

  typedef std::vector<FieldEntry>::iterator iterator;
  typedef std::vector<FieldEntry>::const_iterator const_iterator;

  Message(MessageDescription::Ptr description);
  Message(const Message& other);
  Message& operator=(const Message& other);
  ~Message();

  bool operator==(const Message& other) const;
  bool operator!=(const Message& other) const {
    return !operator==(other);
  }

  bool hasField(const std::string& name);

  MessageValue& operator[](const std::string& name);

  template<typename Stream>
  void read(Stream& stream);

  template<typename Stream>
  void write(Stream& stream) const;

  void morph(MessageDescription::Ptr description);

  MessageDescription::Ptr getDescription() const {
    return description_;
  }

  const_iterator begin() const { return fields_.begin(); }
  const_iterator end() const { return fields_.end(); }
  iterator begin() { return fields_.begin(); }
  iterator end() { return fields_.end(); }

private:
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
