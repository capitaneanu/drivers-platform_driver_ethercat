#include <iostream>

#include "CanDeviceAtiFts.h"
#include "CanOverEthercat.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDeviceAtiFts::CanDeviceAtiFts(CanOverEthercat* can_interface,
                                 unsigned int can_id,
                                 std::string device_name)
    : CanDevice(can_interface, can_id, device_name),
      _input(NULL),
      _output(NULL),
      _counts_per_force(1),
      _counts_per_torque(1),
      _force_bias(0, 0, 0),
      _torque_bias(0, 0, 0)

{
    if (device_name == "FTS_FL")
    {
        _force_bias = Eigen::Vector3d(0, 0, 0);
        _torque_bias = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_FR")
    {
        _force_bias = Eigen::Vector3d(0, 0, 0);
        _torque_bias = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_CL")
    {
        _force_bias = Eigen::Vector3d(0, 0, 0);
        _torque_bias = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_CR")
    {
        _force_bias = Eigen::Vector3d(0, 0, 0);
        _torque_bias = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_BL")
    {
        _force_bias = Eigen::Vector3d(0, 0, 0);
        _torque_bias = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_BR")
    {
        _force_bias = Eigen::Vector3d(0, 0, 0);
        _torque_bias = Eigen::Vector3d(0, 0, 0);
    }
}

CanDeviceAtiFts::~CanDeviceAtiFts() {}

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

    // sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x00});

    bool success = true;

    for (auto sdo_write : sdo_writes)
    {
        success &= _can_interface->sdoWrite(
            _can_id, sdo_write.index, sdo_write.subindex, sdo_write.fieldsize, sdo_write.data);
    }

    int force_unit;
    int torque_unit;

    success &= _can_interface->sdoRead(_can_id, DictionaryObject::CALIBRATION, 0x29, &force_unit);
    success &= _can_interface->sdoRead(_can_id, DictionaryObject::CALIBRATION, 0x2a, &torque_unit);

    LOG_DEBUG_S << "Force unit of sensor " << _device_name << " is " << force_unit;
    LOG_DEBUG_S << "Torque unit of sensor " << _device_name << " is " << torque_unit;

    success &=
        _can_interface->sdoRead(_can_id, DictionaryObject::CALIBRATION, 0x31, &_counts_per_force);
    success &=
        _can_interface->sdoRead(_can_id, DictionaryObject::CALIBRATION, 0x32, &_counts_per_torque);

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

void CanDeviceAtiFts::setInputPdo(unsigned char* input_pdo) { _input = (TxPdo*)input_pdo; }

void CanDeviceAtiFts::setOutputPdo(unsigned char* output_pdo)
{
    _output = (RxPdo*)output_pdo;

    _output->control_1 = 0x00000000;
    _output->control_2 = 0x00000000;
}

bool CanDeviceAtiFts::startup() { return true; }

bool CanDeviceAtiFts::shutdown() { return true; }

bool CanDeviceAtiFts::reset() { return shutdown() && startup(); }

Eigen::Vector3d CanDeviceAtiFts::readForceN()
{
    double fx = _input->fx * 1.0 / _counts_per_force;
    double fy = _input->fy * 1.0 / _counts_per_force;
    double fz = _input->fz * 1.0 / _counts_per_force;

    Eigen::Vector3d force = Eigen::Vector3d(fx, fy, fz);
    force -= _force_bias;

    return force;
}

Eigen::Vector3d CanDeviceAtiFts::readTorqueNm()
{
    double tx = _input->tx * 1.0 / _counts_per_torque;
    double ty = _input->ty * 1.0 / _counts_per_torque;
    double tz = _input->tz * 1.0 / _counts_per_torque;

    Eigen::Vector3d torque = Eigen::Vector3d(tx, ty, tz);
    torque -= _torque_bias;

    return torque;
}

bool CanDeviceAtiFts::isError() { return false; }

unsigned int CanDeviceAtiFts::getError() { return 0; }
