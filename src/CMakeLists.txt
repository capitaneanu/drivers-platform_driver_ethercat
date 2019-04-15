include_directories(${CMAKE_INSTALL_PREFIX}/include/soem)

rock_library(platform_driver
    SOURCES CanOverEthercat.cpp CanDriveTwitter.cpp Platform_Driver.cpp
    HEADERS CanOverEthercat.h CanDriveTwitter.h DriveParam.h Mutex.h Platform_Driver.h
    DEPS_PKGCONFIG soem)
