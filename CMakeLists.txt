if (DEFINED ENV{ROCK_PREFIX})
  project(platform_driver_ethercat)
  cmake_minimum_required(VERSION 2.6)

  find_package(Rock)
  rock_init(${PROJECT_NAME} 0.1)
  rock_standard_layout()

  add_definitions(-Wall)
  add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME}_lib)
  add_definitions(-DBASE_LOG_DEBUG)
  add_definitions(-DROCK)

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

  file(GLOB MY_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)
  file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)

  include_directories(include)

  rock_library(
    ${PROJECT_NAME}
    SOURCES ${MY_SOURCES}
    HEADERS ${MY_HEADERS}
    DEPS_PKGCONFIG soem base-lib
  )

elseif(DEFINED ENV{ROS_DISTRO})
  cmake_minimum_required(VERSION 3.5)
  project(platform_driver_ethercat_lib)

  add_definitions(-DROS2)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(PkgConfig REQUIRED)
  find_package(rclcpp REQUIRED)
  pkg_check_modules(SOEM REQUIRED IMPORTED_TARGET soem)

  # add library
  file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
  add_library(${PROJECT_NAME} STATIC ${MY_SOURCES})

  # specify include paths and dependencies
  target_include_directories(${PROJECT_NAME} PRIVATE src PRIVATE include)
  target_link_libraries(${PROJECT_NAME} PkgConfig::SOEM Eigen3::Eigen)
  ament_target_dependencies(${PROJECT_NAME} rclcpp)

  # copy public headers to destination
  install(
    DIRECTORY include/
    DESTINATION include
  )

  # install target at destination
  install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  # export information for upstream packages
  ament_export_libraries(${PROJECT_NAME})
  ament_export_include_directories(include)
  ament_export_dependencies(Eigen3)

  ament_package()

endif()
