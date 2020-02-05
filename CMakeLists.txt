cmake_minimum_required(VERSION 3.5)

if (DEFINED ENV{ROCK_PREFIX})
    include(Rock.cmake)
elseif(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)
    include(Ros2.cmake)
endif()
