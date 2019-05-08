rock_library(platform_driver
    SOURCES CanOverEthercat.cpp CanDevice.cpp CanDeviceAtiFts.cpp CanDriveTwitter.cpp Platform_Driver.cpp
    HEADERS CanOverEthercat.h CanDevice.h CanDeviceAtiFts.h CanDriveTwitter.h DriveParam.h Platform_Driver.h
    DEPS_PKGCONFIG soem base-lib)
