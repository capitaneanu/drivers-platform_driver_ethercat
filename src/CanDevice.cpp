#include <iostream>

#include "CanDevice.h"

using namespace platform_driver_ethercat;

CanDevice::CanDevice(std::shared_ptr<EthercatInterface> ethercat, unsigned int slave_id, std::string device_name)
    : ethercat_(std::move(ethercat)), slave_id_(slave_id), device_name_(device_name)
{
}

CanDevice::~CanDevice() {}

unsigned int CanDevice::getSlaveId() { return slave_id_; }

std::string CanDevice::getDeviceName() { return device_name_; }
