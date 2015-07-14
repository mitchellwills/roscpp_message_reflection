#include <gtest/gtest.h>
#include <roscpp_message_reflection/message_value.h>
#include <roscpp_message_reflection/message_impl.h>

using namespace roscpp_message_reflection;

std::string generate_string_test_value(size_t i) {
  std::stringstream ss;
  ss << "A test string " << i;
  return ss.str();
}

bool generate_bool_test_value(size_t i) {
  return (i % 2)==0 ? false : true;
}

template <typename T>
T generate_numeric_test_value(size_t i) {
  return (T)(std::numeric_limits<T>::min() +  (std::numeric_limits<T>::max() - std::numeric_limits<T>::min()) * ((i % 34) / 34.0d));
}

MessageDescription::Ptr create_test_message_description() {
  std::vector<FieldDescription> fields;
  fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  return MessageDescription::Ptr(new MessageDescription("a_package/AMessage", "992ce8a1687cec8c8bd883ec73ca1234", "int32 a\nuint16 b\nstring d\n\n", fields, std::map<std::string, MessageDescription::Ptr>()));
}
MessageDescription::Ptr test_message_description() {
  static MessageDescription::Ptr type(create_test_message_description());
  return type;
}

Message generate_test_message(size_t i) {
  Message m(test_message_description());
  m["a"] = i % 9;
  m["b"] = i % 11;
  m["d"] = generate_string_test_value(i % 13);
  return m;
}

MessageValue generate_test_message_value() {
  return MessageValue::Create(test_message_description());
}

MessageValue generate_test_message_vl_array() {
  return MessageValue::CreateVariableLengthArray(test_message_description());
}

MessageValue generate_test_message_fl_array(size_t size) {
  return MessageValue::CreateFixedLengthArray(test_message_description(), size);
}


#define DECLARE_VALUE_TYPE_TEST(label, value_generator, expected_type)	\
  TEST(MessageValueTest, get_type_##label) {				\
    MessageValue value = value_generator();				\
    EXPECT_EQ(MessageValue::expected_type, value.valueType());		\
    EXPECT_FALSE(value.isArray());					\
  }

#define DECLARE_ARRAY_TYPE_TEST(label, array_generator, expected_value_type) \
  TEST(MessageValueTest, get_array_type_##label) {			\
    MessageValue value = array_generator();				\
    EXPECT_EQ(MessageValue::expected_value_type, value.valueType());	\
    EXPECT_TRUE(value.isArray());					\
  }

#define DECLARE_ASSIGN_AND_RETRIEVE_GET_TEST(label, value_generator, value_type, test_value) \
  TEST(MessageValueTest, assign_and_retrieve_get_##label) {		\
    MessageValue value = value_generator();				\
    value = test_value;							\
    EXPECT_EQ(test_value, value.get<value_type>());			\
  }

#define DECLARE_ASSIGN_AND_RETRIEVE_AS_TEST(label, value_generator, value_type, test_value) \
  TEST(MessageValueTest, assign_and_retrieve_as_##label) {		\
    MessageValue value = value_generator();				\
    value = test_value;							\
    EXPECT_EQ(test_value, value.as<value_type>());			\
  }


#define DECLARE_ASSIGN_AND_RETRIEVE_VL_ARRAY_TEST(label, value_type, array_generator, value_generator) \
  TEST(MessageValueTest, assign_and_retrieve_variable_length_array_of_##label) { \
    MessageValue value = array_generator();				\
    value.resize(10);							\
    ASSERT_EQ(10, value.size());					\
    for(int i = 0; i < 10; ++i) {					\
      value.get<value_type>(i) = value_generator(i);			\
    }									\
    for(int i = 0; i < 10; ++i) {					\
      EXPECT_EQ(value_generator(i), value.get<value_type>(i));		\
    }									\
  }

#define DECLARE_ASSIGN_AND_RETRIEVE_FL_ARRAY_TEST(label, value_type, array_generator, value_generator) \
  TEST(MessageValueTest, assign_and_retrieve_fixed_length_array_of_##label) { \
    MessageValue value = array_generator(10);				\
    ASSERT_EQ(10, value.size());					\
    for(int i = 0; i < 10; ++i) {					\
      value.get<value_type>(i) = value_generator(i);			\
    }									\
    for(int i = 0; i < 10; ++i) {					\
      EXPECT_EQ(value_generator(i), value.get<value_type>(i));		\
    }									\
  }									\
  TEST(MessageValueTest, resize_fixed_length_array_of_##label) {	\
    MessageValue value = array_generator(10);				\
    EXPECT_THROW({							\
	value.resize(11);						\
      }, MessageException);						\
    EXPECT_EQ(10, value.size());					\
  }

#define DECLARE_ASSIGN_AS_AND_RETRIEVE_AS_TEST(label, value_generator, value_type, other_type, test_value) \
  TEST(MessageValueTest, assign_and_retrieve_##label) {			\
    MessageValue value = value_generator();				\
    value.operator=<other_type>(test_value);				\
    EXPECT_EQ(test_value, value.as<other_type>());			\
  }

#define DECLARE_MISMATCH_ASSIGN_TEST(label, dest_value_generator, dest_type, dest_type_value, \
				     src_type, src_type_value)		\
  TEST(MessageValueTest, mismatched_type_assign_##label) {		\
    MessageValue value = dest_value_generator();			\
    value.operator=<dest_type>(dest_type_value);			\
    EXPECT_THROW({							\
	value.operator=<src_type>(src_type_value);			\
      }, MessageException);						\
    EXPECT_EQ(dest_type_value, value.get<dest_type>());			\
  }
