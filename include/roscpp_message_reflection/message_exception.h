#ifndef ROSCPP_MESSAGE_REFLECTION_MESSAGE_EXCEPTION_H
#define ROSCPP_MESSAGE_REFLECTION_MESSAGE_EXCEPTION_H

#include <string>
#include <ros/exception.h>

namespace roscpp_message_reflection {

class MessageException : public ros::Exception {
public:
  MessageException(const std::string& msg)
    : ros::Exception(msg)  {}
};

}

#endif
