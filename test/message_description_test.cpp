#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include "test_utils.h"
#include <roscpp_message_reflection/message_description_provider.h>

// NOTE Since comments in message full text varies between distros (even with same hash)
//      we just check that the full text is not empty

using namespace roscpp_message_reflection;

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
  MessageDescription description("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>());
  EXPECT_EQ("a_package/AMessage", description.name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ec73ca1234", description.md5sum);
  EXPECT_GT(description.full_text.size(), 0);
  ASSERT_EQ(3, description.fields.size());
  EXPECT_VALUE_FIELD("a", "int32", description.fields[0]);
  EXPECT_VALUE_FIELD("b", "uint16", description.fields[1]);
  EXPECT_VALUE_FIELD("d", "string", description.fields[2]);
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
  MessageDescription::Ptr description = provider->getMessageDescription("std_msgs/String");
  ASSERT_TRUE(description);
  EXPECT_EQ("std_msgs/String", description->name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ec73ca41d1", description->md5sum);
  EXPECT_GT(description->full_text.size(), 0);

  ASSERT_EQ(1, description->fields.size());
  EXPECT_VALUE_FIELD("data", "string", description->fields[0]);
}

TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_Vector3_description)
{
  MessageDescription::Ptr description = provider->getMessageDescription("geometry_msgs/Vector3");
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
TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_PoseWithCovariance_description)
{
  MessageDescription::Ptr description = provider->getMessageDescription("geometry_msgs/PoseWithCovariance");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/PoseWithCovariance", description->name);
  EXPECT_EQ("c23e848cf1b7533a8d7c259073a97e6f", description->md5sum);
  EXPECT_GT(description->full_text.size(), 0);

  ASSERT_EQ(2, description->fields.size());
  EXPECT_VALUE_FIELD("pose", "geometry_msgs/Pose", description->fields[0]);
  EXPECT_FIXED_LENGTH_ARRAY_FIELD("covariance", "float64", 36, description->fields[1]);
}

// Tests expansion of type without package recursivly
TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_PoseWithCovarianceStamped_description)
{
  MessageDescription::Ptr description = provider->getMessageDescription("geometry_msgs/PoseWithCovarianceStamped");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/PoseWithCovarianceStamped", description->name);
  EXPECT_EQ("953b798c0f514ff060a53a3498ce6246", description->md5sum);
  EXPECT_GT(description->full_text.size(), 0);

  ASSERT_EQ(2, description->fields.size());
  EXPECT_VALUE_FIELD("header", "std_msgs/Header", description->fields[0]);
  EXPECT_VALUE_FIELD("pose", "geometry_msgs/PoseWithCovariance", description->fields[1]);
}


// Tests variable length array
TEST_F(MessageDescriptionProviderTest, get_geometry_msgs_PoseArray_description)
{
  MessageDescription::Ptr description = provider->getMessageDescription("geometry_msgs/PoseArray");
  ASSERT_TRUE(description);
  EXPECT_EQ("geometry_msgs/PoseArray", description->name);
  EXPECT_EQ("916c28c5764443f268b296bb671b9d97", description->md5sum);
  EXPECT_GT(description->full_text.size(), 0);

  ASSERT_EQ(2, description->fields.size());
  EXPECT_VALUE_FIELD("header", "std_msgs/Header", description->fields[0]);
  EXPECT_VARIABLE_LENGTH_ARRAY_FIELD("poses", "geometry_msgs/Pose", description->fields[1]);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "message_description_test");

  return RUN_ALL_TESTS();
}
