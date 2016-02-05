
include_directories($ENV{CMAKE_PREFIX_PATH}/include)

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "dir='${dir}'")
#endforeach()

rock_library(platform_driver
    SOURCES CanPeakSysUSB.cpp CanDriveWhistle.cpp StrUtil.cpp TimeStamp.cpp Platform_Driver.cpp
    HEADERS CanMsg.h CanItf.h CanPeakSysUSB.h CanDriveItf.h CanDriveWhistle.h DriveParam.h Mutex.h StrUtil.h TimeStamp.h Platform_Driver.h)

#rock_executable(platform_driver_bin main.cpp
#    DEPS platform_driver)

