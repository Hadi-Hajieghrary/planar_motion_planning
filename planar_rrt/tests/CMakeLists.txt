include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
)
# Make Google Test available
FetchContent_MakeAvailable(googletest)

add_executable(utility_test utility_test.cpp)
target_link_libraries(utility_test gtest gtest_main libutility)
add_test(NAME UtilityTests COMMAND utility_test)

add_executable(robot_test robot_test.cpp)
target_link_libraries(robot_test gtest gtest_main librobot)
add_test(NAME RobotTests COMMAND robot_test)