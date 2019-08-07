#include <iostream>

#include "CanDevice.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDevice::CanDevice(CanOverEthercat* can_interface, unsigned int can_id, std::string device_name)
    : _can_interface(can_interface), _can_id(can_id), _device_name(device_name)
{
}

CanDevice::~CanDevice() {}

unsigned int CanDevice::getCanId() { return _can_id; }

std::string CanDevice::getDeviceName() { return _device_name; }
