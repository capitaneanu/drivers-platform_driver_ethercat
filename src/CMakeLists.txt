project(platform_driver)

rock_library(platform_driver
    SOURCES CanOverEthercat.cpp CanDevice.cpp CanDeviceAtiFts.cpp CanDriveTwitter.cpp Platform_Driver.cpp
    HEADERS CanOverEthercat.h CanDevice.h CanDeviceAtiFts.h CanDriveTwitter.h DriveParam.h Platform_Driver.h
    DEPS_PKGCONFIG soem base-lib)

add_definitions(-Wall)
add_definitions(-DBASE_LOGGING_NAMESPACE=${PROJECT_NAME}_lib)
add_definitions(-DBASE_LOG_INFO)
