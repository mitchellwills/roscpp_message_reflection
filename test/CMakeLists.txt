if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  link_directories(${GTEST_LIBRARY_DIRS})

  add_executable(message_description_test EXCLUDE_FROM_ALL message_description_test.cpp)
  target_link_libraries(message_description_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  #set_target_properties(message_description_test PROPERTIES COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
  #target_link_libraries(message_description_test gcov)

  add_executable(message_serialization_test EXCLUDE_FROM_ALL message_serialization_test.cpp)
  target_link_libraries(message_serialization_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  add_executable(message_test EXCLUDE_FROM_ALL message_test.cpp)
  target_link_libraries(message_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/message_value_test_generated.cpp
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gen_message_value_test.py ${CMAKE_CURRENT_BINARY_DIR}/message_value_test_generated.cpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/gen_message_value_test.py
    )
  add_executable(message_value_test EXCLUDE_FROM_ALL ${CMAKE_CURRENT_BINARY_DIR}/message_value_test_generated.cpp message_value_test.cpp)
  target_link_libraries(message_value_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  if(TARGET tests)
    add_dependencies(tests
      message_description_test
      message_serialization_test
      message_value_test
      message_test
      )
  endif()

endif()

add_rostest(all.test)
