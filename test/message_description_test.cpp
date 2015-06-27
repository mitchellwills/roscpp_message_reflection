#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <roscpp_message_reflection/message_description_provider.h>

using namespace roscpp_message_reflection;

#define EXPECT_VALUE_FIELD(expected_name, expected_type, field)	\
  EXPECT_EQ(expected_name, field.name());			\
  EXPECT_EQ(expected_type, field.full_type());			\
  EXPECT_EQ(expected_type, field.value_type());			\
  EXPECT_EQ(FieldDescription::Value, field.type())

#define EXPECT_VARIABLE_LENGTH_ARRAY_FIELD(expected_name, expected_value_type, field) \
  EXPECT_EQ(expected_name, field.name());				\
  {									\
    std::stringstream expected_full_type_ss;				\
    expected_full_type_ss << expected_value_type;			\
    expected_full_type_ss << "[]";					\
    EXPECT_EQ(expected_full_type_ss.str(), field.full_type());		\
  }									\
  EXPECT_EQ(expected_value_type, field.value_type());			\
  EXPECT_EQ(FieldDescription::VariableLengthArray, field.type());	\

#define EXPECT_FIXED_LENGTH_ARRAY_FIELD(expected_name, expected_value_type, expected_length, field) \
  EXPECT_EQ(expected_name, field.name());				\
  {									\
    std::stringstream expected_full_type_ss;				\
    expected_full_type_ss << expected_value_type;			\
    expected_full_type_ss << "[" << expected_length << "]";		\
    EXPECT_EQ(expected_full_type_ss.str(), field.full_type());		\
  }									\
  EXPECT_EQ(expected_value_type, field.value_type());			\
  EXPECT_EQ(FieldDescription::FixedLengthArray, field.type());		\
  EXPECT_EQ(expected_length, field.array_length())

TEST(MessageDescription, construct_value_field)
{
  FieldDescription field = FieldDescription::CreateFromFullType("some_name", "AType");
  EXPECT_VALUE_FIELD("some_name", "AType", field);
}

TEST(MessageDescription, construct_variable_length_array_field)
{
  FieldDescription field = FieldDescription::CreateFromFullType("some_name", "AType[]");
  EXPECT_VARIABLE_LENGTH_ARRAY_FIELD("some_name", "AType", field);
}

TEST(MessageDescription, construct_fixed_length_array_field)
{
  FieldDescription field = FieldDescription::CreateFromFullType("some_name", "AType[10]");
  EXPECT_FIXED_LENGTH_ARRAY_FIELD("some_name", "AType", 10, field);
}

TEST(MessageDescription, construct_simple_message)
{
  std::vector<FieldDescription> fields;
  fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  MessageDescription message("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>());
  EXPECT_EQ("a_package/AMessage", message.name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ec73ca1234", message.md5sum);
  EXPECT_EQ("int32 a\nuint16 b\nstring d\n\n", message.full_text);
  ASSERT_EQ(3, message.fields.size());
  EXPECT_VALUE_FIELD("a", "int32", message.fields[0]);
  EXPECT_VALUE_FIELD("b", "uint16", message.fields[1]);
  EXPECT_VALUE_FIELD("d", "string", message.fields[2]);
}



class MessageDescriptionProviderTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    provider.reset(MessageDescriptionProvider::Create(nh));
  }
  ros::NodeHandle nh;
  boost::scoped_ptr<MessageDescriptionProvider> provider;
};

TEST_F(MessageDescriptionProviderTest, get_std_msgs_String_description)
{
  MessageDescription::Ptr description = provider->getDescription("std_msgs/String");
  ASSERT_TRUE(description);
  EXPECT_EQ("std_msgs/String", description->name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ec73ca41d1", description->md5sum);
  EXPECT_EQ("string data\n\n", description->full_text);

  ASSERT_EQ(1, description->fields.size());
  EXPECT_VALUE_FIELD("data", "string", description->fields[0]);
}

TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_Vector3_description)
{
  MessageDescription::Ptr description = provider->getDescription("geometry_msgs/Vector3");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/Vector3", description->name);
  EXPECT_EQ("4a842b65f413084dc2b10fb484ea7f17", description->md5sum);
  EXPECT_EQ("# This represents a vector in free space. \n\nfloat64 x\nfloat64 y\nfloat64 z\n", description->full_text);

  ASSERT_EQ(3, description->fields.size());
  EXPECT_VALUE_FIELD("x", "float64", description->fields[0]);
  EXPECT_VALUE_FIELD("y", "float64", description->fields[1]);
  EXPECT_VALUE_FIELD("z", "float64", description->fields[2]);
}

// Tests expansion of type without package and fixed length array
TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_AccelWithCovariance_description)
{
  MessageDescription::Ptr description = provider->getDescription("geometry_msgs/AccelWithCovariance");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/AccelWithCovariance", description->name);
  EXPECT_EQ("ad5a718d699c6be72a02b8d6a139f334", description->md5sum);
  EXPECT_EQ("# This expresses acceleration in free space with uncertainty.\n\nAccel accel\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n\nfloat64 x\nfloat64 y\nfloat64 z\n", description->full_text);

  ASSERT_EQ(2, description->fields.size());
  EXPECT_VALUE_FIELD("accel", "geometry_msgs/Accel", description->fields[0]);
  EXPECT_FIXED_LENGTH_ARRAY_FIELD("covariance", "float64", 36, description->fields[1]);
}

// Tests expansion of type without package recursivly
TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_AccelWithCovarianceStamped_description)
{
  MessageDescription::Ptr description = provider->getDescription("geometry_msgs/AccelWithCovarianceStamped");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/AccelWithCovarianceStamped", description->name);
  EXPECT_EQ("96adb295225031ec8d57fb4251b0a886", description->md5sum);
  EXPECT_EQ("# This represents an estimated accel with reference coordinate frame and timestamp.\nHeader header\nAccelWithCovariance accel\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\n# 0: no frame\n# 1: global frame\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/AccelWithCovariance\n# This expresses acceleration in free space with uncertainty.\n\nAccel accel\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n\nfloat64 x\nfloat64 y\nfloat64 z\n", description->full_text);

  ASSERT_EQ(2, description->fields.size());
  EXPECT_VALUE_FIELD("header", "std_msgs/Header", description->fields[0]);
  EXPECT_VALUE_FIELD("accel", "geometry_msgs/AccelWithCovariance", description->fields[1]);
}


// Tests variable length array
TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_PoseArray_description)
{
  MessageDescription::Ptr description = provider->getDescription("geometry_msgs/PoseArray");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/PoseArray", description->name);
  EXPECT_EQ("916c28c5764443f268b296bb671b9d97", description->md5sum);
  EXPECT_EQ("# An array of poses with a header for global reference.\n\nHeader header\n\nPose[] poses\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\n# 0: no frame\n# 1: global frame\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of postion and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n", description->full_text);

  ASSERT_EQ(2, description->fields.size());
  EXPECT_VALUE_FIELD("header", "std_msgs/Header", description->fields[0]);
  EXPECT_VARIABLE_LENGTH_ARRAY_FIELD("poses", "geometry_msgs/Pose", description->fields[1]);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "message_description_test");

  return RUN_ALL_TESTS();
}
