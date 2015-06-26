#include <gtest/gtest.h>
#include <ros/ros.h>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_description_provider.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <geometry_msgs/Vector3.h>

using namespace roscpp_message_reflection;

class MessageSerializationTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    description_provider.reset(MessageDescriptionProvider::Create(nh));
  }

  ros::serialization::IStream createIStream() {
    size_t message_length = serialized_message.num_bytes - (serialized_message.message_start-serialized_message.buf.get());
    return ros::serialization::IStream(serialized_message.message_start, message_length);
  }

  ros::NodeHandle nh;
  boost::scoped_ptr<MessageDescriptionProvider> description_provider;

  ros::SerializedMessage serialized_message;
};

#define DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(message_name, native_type, test_data) \
  TEST_F(MessageSerializationTest, deserialize_std_msgs_##message_name) { \
    std_msgs::message_name input;					\
    input.data = test_data;						\
									\
    serialized_message = ros::serialization::serializeMessage(input);	\
    ros::serialization::IStream stream = createIStream();		\
									\
    Message message;							\
    message.morph(description_provider->getDescription("std_msgs/"#message_name)); \
    ros::serialization::deserializeMessage(serialized_message, message); \
									\
    EXPECT_EQ(input.data, message["data"].as<native_type>());		\
  }									\
									\
  TEST_F(MessageSerializationTest, serialize_std_msgs_##message_name) {	\
    Message message;							\
    message.morph(description_provider->getDescription("std_msgs/"#message_name)); \
    message["data"] = test_data;					\
									\
    serialized_message = ros::serialization::serializeMessage(message);	\
									\
    std_msgs::message_name output;					\
    ros::serialization::deserializeMessage(serialized_message, output);	\
									\
    EXPECT_EQ(test_data, output.data);					\
  }


DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Bool, uint8_t, true);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Byte, int8_t, 0x3);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Char, uint8_t, 'h');
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Int8, int8_t, -2);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(UInt8, uint8_t, 5);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Int16, int16_t, -200);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(UInt16, uint16_t, 60000);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Int32, int32_t, -234344309);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(UInt32, uint32_t, 24353435);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Int64, int64_t, -1342242323);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(UInt64, uint64_t, 24342422342432);
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(String, std::string, "hello this is a test");
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Time, ros::Time, ros::Time(100));
DECLARE_STD_MSGS_DATA_MESSAGE_TESTS(Duration, ros::Duration, ros::Duration(50));

TEST_F(MessageSerializationTest, deserialize_geometry_msgs_Vector3) {
  geometry_msgs::Vector3 input;
  input.x = 1.5;
  input.y = 2.0;
  input.z = 3.3;

  serialized_message = ros::serialization::serializeMessage(input);

  Message message;
  message.morph(description_provider->getDescription("geometry_msgs/Vector3"));
  ros::serialization::deserializeMessage(serialized_message, message);

  EXPECT_EQ(input.x, message["x"].as<double>());
  EXPECT_EQ(input.y, message["y"].as<double>());
  EXPECT_EQ(input.z, message["z"].as<double>());
}



TEST_F(MessageSerializationTest, serialize_geometry_msgs_Vector3) {
  Message message;
  message.morph(description_provider->getDescription("geometry_msgs/Vector3"));
  message["x"] = 1.5;
  message["y"] = 2.0;
  message["z"] = 3.3;

  serialized_message = ros::serialization::serializeMessage(message);

  geometry_msgs::Vector3 output;
  ros::serialization::deserializeMessage(serialized_message, output);

  EXPECT_EQ(1.5, output.x);
  EXPECT_EQ(2.0, output.y);
  EXPECT_EQ(3.3, output.z);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "message_serialization_test");

  return RUN_ALL_TESTS();
}

