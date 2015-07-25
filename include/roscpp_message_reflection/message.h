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

  const std::string& getDataType() const;
  const std::string& getMD5Sum() const;
  const std::string& getMessageDefinition() const;

  bool hasField(const std::string& name);

  MessageValue& operator[](const std::string& name);
  const MessageValue& operator[](const std::string& name) const;

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

#if !ROS_VERSION_MINIMUM(1, 10, 0) // Hydro and earlier
public:
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
#endif
private:
  MessageDescription::Ptr description_;
  std::vector<FieldEntry> fields_;

};

}

namespace ros {
namespace message_traits {

  template <> struct IsMessage<roscpp_message_reflection::Message> : TrueType { };
  template <> struct IsMessage<const roscpp_message_reflection::Message> : TrueType { };

  template<>
  struct MD5Sum<roscpp_message_reflection::Message>
  {
    static const char* value(const roscpp_message_reflection::Message& m) { return m.getMD5Sum().c_str(); }

    // Used statically, a message appears to be of any type
    static const char* value() { return "*"; }
  };

  template<>
  struct DataType<roscpp_message_reflection::Message>
  {
    static const char* value(const roscpp_message_reflection::Message& m) { return m.getDataType().c_str(); }

    // Used statically, a message appears to be of any type
    static const char* value() { return "*"; }
  };

  template<>
  struct Definition<roscpp_message_reflection::Message>
  {
    static const char* value(const roscpp_message_reflection::Message& m) { return m.getMessageDefinition().c_str(); }
  };

}
}

namespace ros {
namespace serialization {

  template<>
  struct PreDeserialize<roscpp_message_reflection::Message>
  {
    static void notify(const PreDeserializeParams<roscpp_message_reflection::Message>& params);
  };

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
