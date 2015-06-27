#ifndef ROSCPP_MESSAGE_REFLECTION_UTIL_H
#define ROSCPP_MESSAGE_REFLECTION_UTIL_H

#include <stdlib.h>
#include <cxxabi.h>
#include <string>

namespace roscpp_message_reflection {

inline std::string demangle(const char* name) {
  int status;
  char* realname = abi::__cxa_demangle(name, 0, 0, &status);
  std::string result(realname);
  free(realname);
  return result;
}

template <typename T>
inline std::string demangle_type() {
  return demangle(typeid(T).name());
}

}

#endif
