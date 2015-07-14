#include <gtest/gtest.h>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_impl.h>

using namespace roscpp_message_reflection;


TEST(Message, construct_message)
{
  std::vector<FieldDescription> fields;
  fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  MessageDescription::Ptr description(new MessageDescription("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>()));

  Message m(description);

  ASSERT_TRUE(m.hasField("a"));
  ASSERT_TRUE(m.hasField("b"));
  ASSERT_TRUE(m.hasField("d"));

  Message::iterator itr = m.begin();
  ASSERT_NE(itr, m.end());
  EXPECT_EQ("a", itr->name);
  ++itr;
  ASSERT_NE(itr, m.end());
  EXPECT_EQ("b", itr->name);
  ++itr;
  ASSERT_NE(itr, m.end());
  EXPECT_EQ("d", itr->name);
  ++itr;
  ASSERT_EQ(itr, m.end());
}


TEST(Message, assign_fields)
{
  std::vector<FieldDescription> fields;
  fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  MessageDescription::Ptr description(new MessageDescription("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>()));

  Message m(description);
  m["a"] = 10;
  m["b"] = 22;
  m["d"] = "a test";

  EXPECT_EQ(10, m["a"].as<int>());
  EXPECT_EQ(22, m["b"].as<int>());
  EXPECT_EQ("a test", m["d"].as<std::string>());
}

TEST(Message, iterate_message_fields)
{
  std::vector<FieldDescription> fields;
  fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  MessageDescription::Ptr description(new MessageDescription("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>()));

  Message m(description);
  m["a"] = 10;
  m["b"] = 22;
  m["d"] = "a test";

  Message::iterator itr = m.begin();
  ASSERT_NE(itr, m.end());
  EXPECT_EQ("a", itr->name);
  EXPECT_EQ(10, itr->value.as<int>());

  ++itr;
  ASSERT_NE(itr, m.end());
  EXPECT_EQ("b", itr->name);
  EXPECT_EQ(22, itr->value.as<int>());

  ++itr;
  ASSERT_NE(itr, m.end());
  EXPECT_EQ("d", itr->name);
  EXPECT_EQ("a test", itr->value.as<std::string>());

  ++itr;
  ASSERT_EQ(itr, m.end());
}

TEST(Message, get_description)
{
  std::vector<FieldDescription> fields;
  fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  MessageDescription::Ptr description(new MessageDescription("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>()));

  Message m(description);
  EXPECT_EQ(description, m.getDescription());
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

