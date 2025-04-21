cmake_minimum_required(VERSION 3.5)
project(strobe_sync_tests)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
)

find_package(strobe_sync_policy REQUIRED)
find_package(GTest REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

add_executable(test_strobe_sync
    test_strobe_sync.cpp
)

target_link_libraries(test_strobe_sync
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    strobe_sync_policy::strobe_sync_policy
)

add_test(NAME test_strobe_sync COMMAND test_strobe_sync)
