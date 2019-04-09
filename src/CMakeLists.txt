include_directories(${CMAKE_INSTALL_PREFIX}/include/soem)

rock_library(platform_driver
    SOURCES CanOverEthercat.cpp CanPeakSysUSB.cpp CanDriveWhistle.cpp StrUtil.cpp TimeStamp.cpp Platform_Driver.cpp
    HEADERS CanMsg.h CanItf.h CanOverEthercat.h CanPeakSysUSB.h CanDriveItf.h CanDriveWhistle.h DriveParam.h Mutex.h StrUtil.h TimeStamp.h Platform_Driver.h
    DEPS_PKGCONFIG soem)
