cmake_minimum_required(VERSION 2.8.3)
project(roscpp_message_reflection)

find_package(catkin REQUIRED COMPONENTS
  roscpp_serialization
  topic_tools
  message_generation
  cpp_common
  rospy
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
   FILES
   MessageInfo.msg
   MessageFieldInfo.msg
)

add_service_files(
   FILES
   GetMessageInfo.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscpp_message_reflection
  CATKIN_DEPENDS roscpp_serialization topic_tools cpp_common message_runtime rospy roscpp
)

###########
## Build ##
###########

# TODO either need to figure out a better way to do this or pass it down to dependants so it is inherited
add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=30)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(roscpp_message_reflection
  src/message.cpp
  src/message_value.cpp
  src/message_description_provider.cpp
)
target_link_libraries(roscpp_message_reflection
  ${catkin_LIBRARIES}
)
add_dependencies(roscpp_message_reflection roscpp_message_reflection_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS
  scripts/message_description_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS roscpp_message_reflection
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
