#include <unistd.h>
#include <iostream>

#include "CanDriveTwitter.h"
#include "CanOverEthercat.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDriveTwitter::CanDriveTwitter(CanOverEthercat& can_interface,
                                 unsigned int slave_id,
                                 std::string device_name,
                                 DriveConfig drive_config,
                                 bool enabled)
    : CanDevice(can_interface, slave_id, device_name),
      drive_param_(drive_config),
      enabled_(enabled),
      input_(NULL),
      output_(NULL)
{
}

CanDriveTwitter::~CanDriveTwitter() {}

bool CanDriveTwitter::configure()
{
    LOG_DEBUG_S << "Configuring drive " << device_name_ << " ...";

    typedef struct SdoWrite
    {
        uint16_t index;
        uint8_t subindex;
        uint8_t fieldsize;
        int32_t data;
    } SdoWrite;

    std::vector<SdoWrite> sdo_writes;

    // set RxPDO map
    sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x00});    // disable
    sdo_writes.push_back(SdoWrite{0x1c12, 1, 2, 0x160a});  // control word
    sdo_writes.push_back(SdoWrite{0x1c12, 2, 2, 0x160b});  // mode of operation
    sdo_writes.push_back(SdoWrite{0x1c12, 3, 2, 0x160f});  // target position
    sdo_writes.push_back(SdoWrite{0x1c12, 4, 2, 0x161c});  // target velocity
    sdo_writes.push_back(SdoWrite{0x1c12, 5, 2, 0x160c});  // target torque
    sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x05});    // enable

    // set TxPDO map
    sdo_writes.push_back(SdoWrite{0x1c13, 0, 1, 0x00});    // disable
    sdo_writes.push_back(SdoWrite{0x1c13, 1, 2, 0x1a0a});  // status word
    sdo_writes.push_back(SdoWrite{0x1c13, 2, 2, 0x1a0b});  // mode of operation display
    sdo_writes.push_back(SdoWrite{0x1c13, 3, 2, 0x1a0e});  // actual position
    sdo_writes.push_back(SdoWrite{0x1c13, 4, 2, 0x1a11});  // actual velocity
    sdo_writes.push_back(SdoWrite{0x1c13, 5, 2, 0x1a13});  // actual torque
    sdo_writes.push_back(SdoWrite{0x1c13, 6, 2, 0x1a1d});  // analog input
    sdo_writes.push_back(SdoWrite{0x1c13, 0, 1, 0x06});    // enable

    // set commutation
    sdo_writes.push_back(SdoWrite{0x3034, 17, 4, 0x00000003});  // commutation method
    sdo_writes.push_back(
        SdoWrite{0x31d6, 1, 4, 0x41f00000});  // stepper commutation desired current

    // set limits
    sdo_writes.push_back(SdoWrite{0x6072, 0, 2, 0x0c76});  // max torque (from stall torque)

    int rated_current = drive_param_.getNominalCurrent() * 1000.0;
    int max_current;

    if (rated_current == 0)
    {
        max_current = 1;
    }
    else
    {
        max_current = (drive_param_.getCurrMax() * 1000.0 * 1000.0) / rated_current;
    }

    sdo_writes.push_back(
        SdoWrite{0x6073,
                 0,
                 2,
                 max_current});  // max current (from stall current, in thousands of rated current)
    sdo_writes.push_back(SdoWrite{0x6075, 0, 4, rated_current});  // motor rated current (in mNm)
    sdo_writes.push_back(SdoWrite{0x6076, 0, 4, 0x0000000b});     // motor rated torque (11 mNm)
    sdo_writes.push_back(
        SdoWrite{0x607d, 1, 4, (int)drive_param_.getPosMin()});  // min position limit
    sdo_writes.push_back(
        SdoWrite{0x607d, 2, 4, (int)drive_param_.getPosMax()});  // max position limit
    sdo_writes.push_back(
        SdoWrite{0x607f, 0, 4, (int)drive_param_.getVelMax()});  // max profile velocity
    sdo_writes.push_back(
        SdoWrite{0x60c5, 0, 4, (int)drive_param_.getMaxAcc()});  // max acceleration
    sdo_writes.push_back(
        SdoWrite{0x60c6, 0, 4, (int)drive_param_.getMaxDec()});  // max deceleration

    // set profile motion parameters
    sdo_writes.push_back(
        SdoWrite{0x6081, 0, 4, (int)drive_param_.getPtpVelDefault()});  // profile velocity
    sdo_writes.push_back(
        SdoWrite{0x6083, 0, 4, (int)drive_param_.getMaxAcc()});  // profile acceleration
    sdo_writes.push_back(
        SdoWrite{0x6084, 0, 4, (int)drive_param_.getMaxDec()});  // profile deceleration

    // factors (set to 1 to convert units on software side instead of Elmo conversion)
    sdo_writes.push_back(
        SdoWrite{0x608f, 1, 4, 0x00000001});  // position encoder resolution (encoder increments)
    sdo_writes.push_back(
        SdoWrite{0x608f, 2, 4, 0x00000001});  // position encoder resolution (encoder increments)
    sdo_writes.push_back(
        SdoWrite{0x6090, 1, 4, 0x00000001});  // velocity encoder resolution (encoder increments)
    sdo_writes.push_back(
        SdoWrite{0x6090, 2, 4, 0x00000001});  // velocity encoder resolution (motor revolutions)
    sdo_writes.push_back(
        SdoWrite{0x6091, 1, 4, 0x00000001});  // gear ratio (motor shaft revolutions)
    sdo_writes.push_back(
        SdoWrite{0x6091, 2, 4, 0x00000001});  // gear ratio (driving shaft revolutions)
    sdo_writes.push_back(SdoWrite{0x6092, 1, 4, 0x00000001});  // feed constant (feed)
    sdo_writes.push_back(
        SdoWrite{0x6092, 2, 4, 0x00000001});  // feed constant (driving shaft revolutions)
    sdo_writes.push_back(SdoWrite{0x6096, 1, 4, 0x00000001});  // velocity factor (numerator)
    sdo_writes.push_back(SdoWrite{0x6096, 2, 4, 0x00000001});  // velocity factor (divisor)
    sdo_writes.push_back(SdoWrite{0x6097, 1, 4, 0x00000001});  // acceleration factor (numerator)
    sdo_writes.push_back(SdoWrite{0x6097, 2, 4, 0x00000001});  // acceleration factor (divisor)

    bool success = true;

    for (auto sdo_write : sdo_writes)
    {
        success &= can_interface_.sdoWrite(
            slave_id_, sdo_write.index, sdo_write.subindex, sdo_write.fieldsize, sdo_write.data);
    }

    if (success)
    {
        LOG_INFO_S << "Drive " << device_name_ << " configured";
        return true;
    }
    else
    {
        LOG_ERROR_S << "Failed to configure drive " << device_name_;
        return false;
    }
}

void CanDriveTwitter::setInputPdo(unsigned char* input_pdo) { input_ = (TxPdo*)input_pdo; }

void CanDriveTwitter::setOutputPdo(unsigned char* output_pdo)
{
    output_ = (RxPdo*)output_pdo;

    output_->control_word = 0x0004;  // disable quick stop & disable voltage
    output_->operation_mode = 0;
    output_->target_position = 0;
    output_->target_velocity = 0;
    output_->target_torque = 0;
}

bool CanDriveTwitter::startup()
{
    DriveState state = readDriveState();
    int cnt = 100;

    while (state != ST_OPERATION_ENABLE)
    {
        switch (state)
        {
            case ST_FAULT:
                output_->control_word = 0x0080;  // fault reset
                break;
            case ST_QUICK_STOP_ACTIVE:
                output_->control_word = 0x0004;  // disable quick stop
                break;
            case ST_SWITCH_ON_DISABLED:
                output_->control_word = 0x0006;  // enable voltage
                break;
            case ST_READY_TO_SWITCH_ON:
                output_->control_word = 0x0007;  // switch on
                break;
            case ST_SWITCHED_ON:
                output_->control_word = 0x000f;  // enable operation
                break;
            default: break;
        }

        usleep(100000);  // sleep 0.1 s
        state = readDriveState();

        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not start up drive " << device_name_
                        << ". Last state was " << state;
            return false;
        }
    }

    return true;
}

bool CanDriveTwitter::shutdown()
{
    DriveState state = readDriveState();
    int cnt = 100;

    while (state != ST_SWITCH_ON_DISABLED)
    {
        switch (state)
        {
            case ST_OPERATION_ENABLE:
                output_->control_word = 0x0007;  // disable operation
                break;
            case ST_SWITCHED_ON:
                output_->control_word = 0x0006;  // switch off
                break;
            case ST_READY_TO_SWITCH_ON:
                output_->control_word = 0x0004;  // disable voltage
                break;
            case ST_FAULT:
                output_->control_word = 0x0080;  // fault reset
                break;
            case ST_QUICK_STOP_ACTIVE:
                output_->control_word = 0x0004;  // disable quick stop & disable voltage
                break;
            default: break;
        }

        usleep(10000);  // sleep 0.01 s
        state = readDriveState();

        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not shut down drive " << device_name_
                        << ". Last state was " << state;
            return false;
        }
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_ << " shut down.";
    return true;
}

bool CanDriveTwitter::reset() { return shutdown() && startup(); }

CanDriveTwitter::OperationMode CanDriveTwitter::readOperationMode()
{
    return (OperationMode)input_->operation_mode_display;
}

bool CanDriveTwitter::commandOperationMode(CanDriveTwitter::OperationMode mode)
{
    OperationMode current_mode = readOperationMode();

    if (current_mode == mode)
    {
        return true;
    }

    output_->operation_mode = mode;

    int cnt = 1000;

    while (current_mode != mode)
    {
        if (cnt-- == 0)
        {
            LOG_WARN_S << __PRETTY_FUNCTION__ << ": Could not set operation mode for drive "
                       << device_name_ << ". Current mode is " << current_mode
                       << ". Requested mode is " << mode << ".";
            return false;
        }

        usleep(1000);  // sleep 0.001 s
        current_mode = readOperationMode();
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Successfully changed operation mode for drive "
                << device_name_ << " to " << current_mode;

    return true;
}

void CanDriveTwitter::commandPositionRad(double position_rad)
{
    int max_pos = drive_param_.getPosMax();
    int min_pos = drive_param_.getPosMin();
    int target_pos = drive_param_.getSign() * drive_param_.PosGearRadToPosMotIncr(position_rad);
    int target_pos_limited = std::min(max_pos, std::max(min_pos, target_pos));

    if (target_pos != target_pos_limited)
    {
        LOG_WARN_S << "Command exceeds position limit for drive " << device_name_;
    }

    output_->target_position = target_pos_limited;
    commandOperationMode(OM_PROFILE_POSITION);
    // commandOperationMode(OM_CYCSYNC_POSITION);
    output_->control_word |= 0x0030;  // new set point & change set point immediately

    int cnt = 1000;

    while (!checkSetPointAcknowledge())
    {
        usleep(1000);  // sleep 0.001 s

        if (cnt-- == 0)
        {
            LOG_INFO_S << __PRETTY_FUNCTION__ << ": New set point " << output_->target_position
                       << " was not acknowledged for drive " << device_name_;
            break;
        }
    }

    output_->control_word &= 0xffef;  // no new set point

    // LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Drive: " << device_name_ << " Current position: " <<
    // input_->actual_position << " Target position: " << output_->target_position;
}

void CanDriveTwitter::commandVelocityRadSec(double velocity_rad_sec)
{
    int max_vel = drive_param_.getVelMax();
    int target_vel =
        drive_param_.getSign() * drive_param_.VelGearRadSToVelMotIncrPeriod(velocity_rad_sec);
    int target_vel_limited = std::min(max_vel, std::max(-max_vel, target_vel));

    if (target_vel != target_vel_limited)
    {
        LOG_WARN_S << "Command exceeds velocity limit for drive " << device_name_;
    }

    int current_pos = input_->actual_position;
    int max_pos = drive_param_.getPosMax();
    int min_pos = drive_param_.getPosMin();
    int target_vel_poslimited = target_vel_limited;

    if (current_pos >= max_pos)
        target_vel_poslimited = std::min(0, target_vel_limited);
    else if (current_pos <= min_pos)
        target_vel_poslimited = std::max(0, target_vel_limited);

    if (target_vel_limited != target_vel_poslimited)
    {
        LOG_WARN_S << "Position limit reached for drive " << device_name_;
    }

    output_->target_velocity = target_vel_poslimited;
    commandOperationMode(OM_PROFILE_VELOCITY);
}

void CanDriveTwitter::commandTorqueNm(double torque_nm)
{
    commandOperationMode(OM_PROFILE_TORQUE);

    int rated_torque = 11;  // 11 mNm

    // TODO: transform from load to motor torque
    output_->target_torque = drive_param_.getSign() * torque_nm * 1000 * 1000 / rated_torque;
}

bool CanDriveTwitter::checkTargetReached()
{
    unsigned char bit10 = (unsigned char)((input_->status_word >> 10) & 0x0001);

    return (bool)bit10;
}

bool CanDriveTwitter::checkSetPointAcknowledge()
{
    unsigned char bit12 = (unsigned char)((input_->status_word >> 12) & 0x0001);

    return (bool)bit12;
}

double CanDriveTwitter::readPositionRad()
{
    // if (device_name_ == "MAST_PAN")
    //    LOG_DEBUG_S << "Current pos of MAST_PAN: " << input_->actual_position;

    return drive_param_.getSign() * drive_param_.PosMotIncrToPosGearRad(input_->actual_position);
}

double CanDriveTwitter::readVelocityRadSec()
{
    return drive_param_.getSign()
           * drive_param_.VelMotIncrPeriodToVelGearRadS(input_->actual_velocity);
}

double CanDriveTwitter::readTorqueNm()
{
    int rated_torque = 11;  // 11 mNm

    // TODO: transform from motor to load torque
    return drive_param_.getSign() * input_->actual_torque * rated_torque / (1000 * 1000);
}

double CanDriveTwitter::readAnalogInputV() { return input_->analog_input * 1.0 / 1000.0; }

CanDriveTwitter::DriveState CanDriveTwitter::readDriveState()
{
    unsigned char status_lower = (unsigned char)input_->status_word;
    unsigned char bits0to3 = status_lower & 0x0f;
    unsigned char bit5 = (status_lower >> 5) & 0x01;
    unsigned char bit6 = (status_lower >> 6) & 0x01;

    switch (bits0to3)
    {
        case 0x0:
            if (!bit6)
                return ST_NOT_READY_TO_SWITCH_ON;
            else
                return ST_SWITCH_ON_DISABLED;
        case 0x1:
            if (bit5 && !bit6) return ST_READY_TO_SWITCH_ON;
        case 0x3:
            if (bit5 && !bit6) return ST_SWITCHED_ON;
        case 0x7:
            if (bit5 && !bit6)
                return ST_OPERATION_ENABLE;
            else if (!bit5 && !bit6)
                return ST_QUICK_STOP_ACTIVE;
        case 0x8:
            if (!bit6) return ST_FAULT;
        case 0xf:
            if (!bit6) return ST_FAULT_REACTION_ACTIVE;
    }

    LOG_WARN_S << "Drive " << device_name_
               << " in unknown state! Lower byte of status word: " << status_lower;
    return ST_UNKNOWN;
}

bool CanDriveTwitter::isError()
{
    DriveState state = readDriveState();

    return (state == ST_FAULT_REACTION_ACTIVE) || (state == ST_FAULT);
}

unsigned int CanDriveTwitter::getError()
{
    unsigned char status_upper = (unsigned char)(input_->status_word >> 8);

    return status_upper;
}

bool CanDriveTwitter::requestEmergencyStop()
{
    uint16_t control_word = output_->control_word;

    // enable quick stop
    control_word &= 0b111111101111011;
    control_word |= 0b000000000000010;

    output_->control_word = control_word;

    int cnt = 100;

    DriveState state;

    do
    {
        usleep(10000);  // sleep 0.01 s
        state = readDriveState();

        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not emergency stop drive "
                        << device_name_ << ". Last state was " << state;
            return false;
        }
    } while (state != ST_QUICK_STOP_ACTIVE);

    return true;
}

bool CanDriveTwitter::isEnabled()
{
    return enabled_;
}
