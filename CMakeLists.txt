if (DEFINED ENV{ROCK_PREFIX})
    project(platform_driver_ethercat)
    cmake_minimum_required(VERSION 2.6)

    find_package(Rock)
    rock_init(platform_driver_ethercat 0.1)
    rock_standard_layout()

    add_definitions(-Wall)
    add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME}_lib)
    add_definitions(-DBASE_LOG_DEBUG)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

    file(GLOB MY_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.h)
    file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)

    rock_library(platform_driver_ethercat
        SOURCES ${MY_SOURCES}
        HEADERS ${MY_HEADERS}
        DEPS_PKGCONFIG soem base-lib)
elseif(DEFINED ENV{ROS_DISTRO})
endif()
