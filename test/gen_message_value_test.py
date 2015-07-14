#!/usr/bin/env python

import sys
from itertools import chain
import re

class Type(object):
    def __init__(self, name, type, is_numeric, single_test_value=None):
        self.name = name
        self.type = type
        self.value_type_name = name
        self.is_numeric = is_numeric
        self.special_values = {}
        self.equivilent_types = []
        self.single_test_value = single_test_value
        self.value_generator = "MessageValue::Create<" + self.type + ">";
        self.vl_array_generator = "MessageValue::CreateVariableLengthArray<" + self.type + ">";
        self.fl_array_generator = "MessageValue::CreateFixedLengthArray<" + self.type + ">";


class BoolType(Type):
    def __init__(self, name, type):
        super(BoolType, self).__init__(name, type, True)
        self.value_type_name = "uint8"
        self.single_test_value = "false"
        self.special_values["false"] = "false"
        self.special_values["true"] = "true"
        self.test_value_generator = "generate_bool_test_value"

class NormalNumericType(Type):
    def __init__(self, name, type):
        super(NormalNumericType, self).__init__(name, type, True)
        self.single_test_value = "std::numeric_limits<"+self.type+">::max()"
        self.special_values["max"] = "std::numeric_limits<"+self.type+">::max()"
        self.special_values["min"] = "std::numeric_limits<"+self.type+">::min()"
        self.special_values["zero"] = "(("+self.type+")0)"
        self.special_values["one"] = "(("+self.type+")1)"
        self.test_value_generator = "generate_numeric_test_value<"+self.type+">"

class IntegerType(NormalNumericType):
    def __init__(self, name, type, signed):
        super(IntegerType, self).__init__(name, type)
        if signed:
            self.special_values["neg_one"] = "(("+self.type+")-1)"

class FloatType(NormalNumericType):
    def __init__(self, name, type):
        super(FloatType, self).__init__(name, type)
        self.special_values["infinity"] = "std::numeric_limits<"+self.type+">::infinity()"
        self.special_values["neg_infinity"] = "-std::numeric_limits<"+self.type+">::infinity()"

class RosChronoType(Type):
    def __init__(self, name, type):
        super(RosChronoType, self).__init__(name, type, False)
        self.single_test_value = self.type+"(1.1)"
        self.test_value_generator = self.type

STRING_LITERAL_TYPES = [
    Type("const_char_star", "const char*", False, "((const char*)\"A test\")"),
    Type("char_star", "char*", False, "((char*)\"A test string\")"),
    Type("char_literal_array", "char[7]", False, "(\"A test\")"),
]

class StringType(Type):
    def __init__(self):
        super(StringType, self).__init__("string", "std::string", False)
        self.single_test_value = "std::string(\"Some string\")"
        self.special_values["empty_string"] = "std::string()"
        self.equivilent_types.extend(STRING_LITERAL_TYPES)
        self.test_value_generator = "generate_string_test_value"

class MessageType(Type):
    def __init__(self):
        super(MessageType, self).__init__("message", "Message", False)
        self.single_test_value = "generate_test_message(1)"
        self.test_value_generator = "generate_test_message"
        self.value_generator = "generate_test_message_value";
        self.vl_array_generator = "generate_test_message_vl_array";
        self.fl_array_generator = "generate_test_message_fl_array";


CPP_BOOL_TYPE = BoolType("cpp_bool", "bool")
BOOL_TYPE = BoolType("bool", "uint8_t")
BOOL_TYPE.equivilent_types.append(CPP_BOOL_TYPE)

MESSAGE_VALUE_TYPES = list(chain.from_iterable((
    IntegerType("int%d"%i, "int%d_t"%i, False),
    IntegerType("uint%d"%i, "uint%d_t"%i, True)
) for i in [8, 16, 32, 64])) + [
    FloatType("float32", "float"),
    FloatType("float64", "double"),
    BOOL_TYPE
] + [
    RosChronoType("time", "ros::Time"),
    RosChronoType("duration", "ros::Duration"),
    StringType(),
    MessageType()
]

VALUE_TYPES = MESSAGE_VALUE_TYPES + STRING_LITERAL_TYPES + [CPP_BOOL_TYPE]

def main(argv):
    filename = argv[1]

    with open(filename, 'w') as f:
        f.write("#include <message_value_test_generated_util.h>\n\n")

        def writeCall(name, *args):
            f.write(name + "(" + (", ".join(map(lambda x: str(x), args))) + ");\n")

        def writeMismatchAssignTest(src_type, dest_type):
            writeCall("DECLARE_MISMATCH_ASSIGN_TEST", "to_"+dest_type.name+"_from_"+src_type.name,
                      dest_type.value_generator, dest_type.type, dest_type.single_test_value,
                      src_type.type, src_type.single_test_value)

        for type in MESSAGE_VALUE_TYPES:
            f.write("\n// " + type.name + "\n")
            writeCall("DECLARE_VALUE_TYPE_TEST", type.name, type.value_generator, type.value_type_name)
            writeCall("DECLARE_ARRAY_TYPE_TEST", type.name, type.vl_array_generator, type.value_type_name)

            writeCall("DECLARE_ASSIGN_AND_RETRIEVE_GET_TEST", type.name, type.value_generator,
                      type.type, type.single_test_value)
            writeCall("DECLARE_ASSIGN_AND_RETRIEVE_AS_TEST", type.name, type.value_generator,
                      type.type, type.single_test_value)

            writeCall("DECLARE_ASSIGN_AND_RETRIEVE_VL_ARRAY_TEST", type.name,
                      type.type, type.vl_array_generator, type.test_value_generator)

            writeCall("DECLARE_ASSIGN_AND_RETRIEVE_FL_ARRAY_TEST", type.name,
                      type.type, type.fl_array_generator, type.test_value_generator)

            for label, value in type.special_values.iteritems():
                writeCall("DECLARE_ASSIGN_AND_RETRIEVE_GET_TEST", type.name+"_"+label, type.value_generator, type.type, value)
                writeCall("DECLARE_ASSIGN_AND_RETRIEVE_AS_TEST", type.name+"_"+label, type.value_generator, type.type, value)

            for other_type in VALUE_TYPES:
                if other_type is type:
                    continue

                if other_type in type.equivilent_types:
                    writeCall("DECLARE_ASSIGN_AND_RETRIEVE_GET_TEST", type.name+ "_from_" + other_type.name,
                              type.value_generator, type.type, type.single_test_value)
                    for label, value in other_type.special_values.iteritems():
                        writeCall("DECLARE_ASSIGN_AND_RETRIEVE_GET_TEST", type.name+ "_from_" + other_type.name+"_"+label,
                                  type.value_generator, type.type, value)
                elif type.is_numeric:
                    if other_type.is_numeric:
                        # The values zero and one should be valid in all numeric types
                        writeCall("DECLARE_ASSIGN_AS_AND_RETRIEVE_AS_TEST", "zero_to_"+type.name+"_from_"+other_type.name,
                                  type.value_generator, type.type, other_type.type, 0)
                        writeCall("DECLARE_ASSIGN_AS_AND_RETRIEVE_AS_TEST", "one_to_"+type.name+"_from_"+other_type.name,
                                  type.value_generator, type.type, other_type.type, 1)
                    else:
                        writeMismatchAssignTest(other_type, type)
                else:
                    writeMismatchAssignTest(other_type, type)


main(sys.argv)
