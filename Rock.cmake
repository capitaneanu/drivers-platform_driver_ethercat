find_package(Rock)
rock_init(${PROJECT_NAME} 0.1)
rock_standard_layout()

add_definitions(-Wall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME}_lib)
add_definitions(-DBASE_LOG_DEBUG)
add_definitions(-DROCK)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
