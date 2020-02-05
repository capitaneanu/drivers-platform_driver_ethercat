project(platform_driver_ethercat)

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
