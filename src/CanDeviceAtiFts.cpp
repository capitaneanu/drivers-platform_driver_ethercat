#include <iostream>

#include "CanDeviceAtiFts.h"
#include "CanOverEthercat.h"
#include "base-logging/Logging.hpp"

CanDeviceAtiFts::CanDeviceAtiFts(CanOverEthercat *can_interface, unsigned int can_id, std::string device_name)
: CanDevice(can_interface, can_id, device_name), _input(NULL), _output(NULL), _counts_per_force(1), _counts_per_torque(1)

{
}

CanDeviceAtiFts::~CanDeviceAtiFts()
{
}

bool CanDeviceAtiFts::configure()
{
    LOG_DEBUG_S << "Configuring device " << _device_name << " ...";

    typedef struct SdoWrite
    {
        uint16_t index;
        uint8_t subindex;
        uint8_t fieldsize;
        int32_t data;
    } SdoWrite;

    std::vector<SdoWrite> sdo_writes;

    //sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x00});   // disable

    bool success = true;

    for (auto sdo_write : sdo_writes)
    {
        success &= _can_interface->sdoWrite(_can_id, sdo_write.index, sdo_write.subindex, sdo_write.fieldsize, sdo_write.data);
    }

    success &= _can_interface->sdoRead(_can_id, DictionaryObject::CALIBRATION, 0x31, &_counts_per_force);
    success &= _can_interface->sdoRead(_can_id, DictionaryObject::CALIBRATION, 0x32, &_counts_per_torque);

    if (success)
    {
        LOG_INFO_S << "Device " << _device_name << " configured";
        return true;
    }
    else
    {
        LOG_ERROR_S << "Failed to configure device " << _device_name;
        return false;
    }
}

void CanDeviceAtiFts::setInputPdo(unsigned char *input_pdo)
{
    _input = (TxPdo *)input_pdo;
}

void CanDeviceAtiFts::setOutputPdo(unsigned char *output_pdo)
{
    _output = (RxPdo *)output_pdo;

    _output->control_1 = 0x00000000;
    _output->control_2 = 0x00000000;
}

bool CanDeviceAtiFts::startup()
{
	return true;
}

bool CanDeviceAtiFts::shutdown()
{	
	return true;
}

bool CanDeviceAtiFts::reset()
{
	return shutdown() && startup();
}

CanDeviceAtiFts::Force CanDeviceAtiFts::readForceN()
{
    double fx = _input->fx / _counts_per_force;
    double fy = _input->fy / _counts_per_force;
    double fz = _input->fz / _counts_per_force;

    Force force = {fx, fy, fz};

    return force;
}

CanDeviceAtiFts::Torque CanDeviceAtiFts::readTorqueNm()
{
    double tx = _input->tx / _counts_per_force;
    double ty = _input->ty / _counts_per_force;
    double tz = _input->tz / _counts_per_force;

    Torque torque = {tx, ty, tz};

    return torque;
}

bool CanDeviceAtiFts::isError()
{
    return false;
}

unsigned int CanDeviceAtiFts::getError()
{
    return 0;
}
