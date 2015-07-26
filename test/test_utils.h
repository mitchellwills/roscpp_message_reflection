#ifndef ROSCPP_MESSAGE_REFLECTION_TEST_TEST_UTILS_H
#define ROSCPP_MESSAGE_REFLECTION_TEST_TEST_UTILS_H

#include <roscpp_message_reflection/message_description.h>

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

#endif
