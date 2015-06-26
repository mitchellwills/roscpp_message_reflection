#include <gtest/gtest.h>
#include <roscpp_message_reflection/message_value.h>

using namespace roscpp_message_reflection;

#define DECLARE_ARITHMETIC_VALUE_TESTS_false(value_type_name, value_type)
#define DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, other_arithmetic_type) \
  TEST(MessageValueTest, assign_and_retrieve_zero_for_##value_type_name##_as_##other_arithmetic_type) { \
    MessageValue value = MessageValue::Create<value_type>();		\
    value.operator=<other_arithmetic_type>(0);				\
    EXPECT_EQ(0, value.as<other_arithmetic_type>());			\
  }									\
  TEST(MessageValueTest, assign_and_retrieve_one_for_##value_type_name##_as_##other_arithmetic_type) { \
    MessageValue value = MessageValue::Create<value_type>();		\
    value.operator=<other_arithmetic_type>(1);				\
    EXPECT_EQ(1, value.as<other_arithmetic_type>());			\
  }
#define DECLARE_ARITHMETIC_VALUE_TESTS_true(value_type_name, value_type) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, int8_t) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, uint8_t) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, int16_t) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, uint16_t)	\
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, int32_t) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, uint32_t)	\
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, int64_t) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, uint64_t)	\
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, float) \
  DECLARE_ARITHMETIC_VALUE_TESTS_WITH_OTHER_TYPE(value_type_name, value_type, double)

#define DECLARE_VALUE_TESTS(value_type_name, value_type, test_data, is_arithmentic)	\
  TEST(MessageValueTest, assign_and_retrieve_##value_type_name) {	\
    MessageValue value = MessageValue::Create<value_type>();		\
    value = test_data;							\
    EXPECT_EQ(test_data, value.as<value_type>());			\
  }									\
  DECLARE_ARITHMETIC_VALUE_TESTS_##is_arithmentic (value_type_name, value_type)

DECLARE_VALUE_TESTS(int8, int8_t, -2, true);
DECLARE_VALUE_TESTS(uint8, uint8_t, 5, true);
DECLARE_VALUE_TESTS(int16, int16_t, -200, true);
DECLARE_VALUE_TESTS(uint16, uint16_t, 60000, true);
DECLARE_VALUE_TESTS(int32, int32_t, -234344309, true);
DECLARE_VALUE_TESTS(uint32, uint32_t, 24353435, true);
DECLARE_VALUE_TESTS(int64, int64_t, -1342242323, true);
DECLARE_VALUE_TESTS(uint64, uint64_t, 24342422342432, true);
DECLARE_VALUE_TESTS(string_from_char_star, std::string, "hello this is a test", false);
DECLARE_VALUE_TESTS(string, std::string, std::string("hello this is a test"), false);
DECLARE_VALUE_TESTS(time, ros::Time, ros::Time(100), false);
DECLARE_VALUE_TESTS(duration, ros::Duration, ros::Duration(50), false);

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

