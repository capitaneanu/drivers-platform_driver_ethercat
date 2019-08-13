#include <iostream>

#include "CanDevice.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDevice::CanDevice(CanOverEthercat* can_interface, unsigned int slave_id, std::string device_name)
    : can_interface_(can_interface), slave_id_(slave_id), device_name_(device_name)
{
}

CanDevice::~CanDevice() {}

unsigned int CanDevice::getSlaveId() { return slave_id_; }

std::string CanDevice::getDeviceName() { return device_name_; }
