if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  add_executable(message_description_test EXCLUDE_FROM_ALL message_description_test.cpp)
  target_link_libraries(message_description_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  #set_target_properties(message_description_test PROPERTIES COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
  #target_link_libraries(message_description_test gcov)

  add_executable(message_serialization_test EXCLUDE_FROM_ALL message_serialization_test.cpp)
  target_link_libraries(message_serialization_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  if(TARGET tests)
    add_dependencies(tests
      message_description_test
      message_serialization_test
      )
  endif()

endif()

add_rostest(message_description.test)
add_rostest(message_serialization.test)
