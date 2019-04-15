#include <iostream>
#include <unistd.h>

#include "CanDriveTwitter.h"

CanDriveTwitter::CanDriveTwitter(CanOverEthercat *can_interface, unsigned int can_id, std::string drive_name)
: _can_interface(can_interface), _can_id(can_id), _drive_name(drive_name)

{
    input = (TxPdo *) _can_interface->getInputPdoPtr(_can_id);
    output = (RxPdo *) _can_interface->getOutputPdoPtr(_can_id);

    output->control_word = 0x0004; // disable quick stop & disable voltage
    output->operation_mode = 0;
    output->target_position = 0;
    output->target_velocity = 0;
    output->target_torque = 0;
}

CanDriveTwitter::~CanDriveTwitter()
{
}

bool CanDriveTwitter::init()
{
    return true;
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
            output->control_word = 0x0080; // fault reset
            break;
        case ST_QUICK_STOP_ACTIVE:
            output->control_word = 0x0004; // disable quick stop
            break; 
        case ST_SWITCH_ON_DISABLED:
            output->control_word = 0x0006; // enable voltage
            break;
        case ST_READY_TO_SWITCH_ON:
            output->control_word = 0x0007; // switch on
            break;
        case ST_SWITCHED_ON:
            output->control_word = 0x000f; // enable operation
            break;
        default:
            break;
        }
        
        usleep(100000); // sleep 0.1 s
        state = readDriveState();

        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::startup: Could not start up drive " << _drive_name << ". Last state was " << state << std::endl;
            return false;
        }
    }

	std::cout << "CanDriveTwitter::startup: Drive " << _drive_name << " started up." << std::endl;
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
            output->control_word = 0x0007; // disable operation 
            break;
        case ST_SWITCHED_ON:
            output->control_word = 0x0006; // switch off
            break;
        case ST_READY_TO_SWITCH_ON:
            output->control_word = 0x0004; // disable voltage
            break;
        case ST_FAULT:
            output->control_word = 0x0080; // fault reset
            break;
        case ST_QUICK_STOP_ACTIVE:
            output->control_word = 0x0004; // disable quick stop & disable voltage
            break; 
        default:
            break; 
        }
        
        usleep(10000); // sleep 0.01 s
        state = readDriveState();

        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::shutdown: Could not shut down drive " << _drive_name << ". Last state was " << state << std::endl;
            return false;
        }
    }

	std::cout << "CanDriveTwitter::shutdown: Drive " << _drive_name << " shut down." << std::endl;
	return true;
}

bool CanDriveTwitter::reset()
{
	return shutdown() && startup();
}

CanDriveTwitter::OperationMode CanDriveTwitter::readOperationMode()
{
    return (OperationMode) input->operation_mode_display;
}

bool CanDriveTwitter::commandOperationMode(CanDriveTwitter::OperationMode mode)
{
    output->operation_mode = mode; 

    int cnt = 1000;

    OperationMode current_mode = readOperationMode();

    while (current_mode != mode)
    {
        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::setOperationMode: Could not set operation mode for drive " << _drive_name << ". Current mode is " << current_mode << ". Requested mode is " << mode << "." << std::endl;
            return false;
        }

        usleep(1000); // sleep 0.001 s
        current_mode = readOperationMode();
    }

    return true;
}

void CanDriveTwitter::commandPositionRad(double position_rad)
{
    commandOperationMode(OM_PROFILE_POSITION);
    //commandOperationMode(OM_CYCSYNC_POSITION);
    output->target_position = _drive_param.PosGearRadToPosMotIncr(position_rad);
    output->control_word |= 0x0030; // new set point & change set point immediately

    std::cout << "CanDriveTwitter::commandPositionRad: Drive: " << _drive_name << " Current position: " << input->actual_position << " Target position: " << output->target_position << std::endl;
}

void CanDriveTwitter::commandVelocityRadSec(double velocity_rad_sec)
{
    commandOperationMode(OM_PROFILE_VELOCITY);
    output->target_velocity = _drive_param.VelGearRadSToVelMotIncrPeriod(velocity_rad_sec);
}

void CanDriveTwitter::commandTorqueNm(double torque_nm)
{
    commandOperationMode(OM_PROFILE_TORQUE);

    int rated_torque = 11; // 11 mNm
    output->target_torque = torque_nm * 1000 * 1000 / rated_torque;
}

bool CanDriveTwitter::checkTargetReached()
{
    unsigned char bit10 = (unsigned char) ((input->status_word >> 10) & 0x0001);

    return (bool) bit10;
}

double CanDriveTwitter::readPositionRad()
{
    return _drive_param.PosMotIncrToPosGearRad(input->actual_position);
}

double CanDriveTwitter::readVelocityRadSec()
{
    return _drive_param.VelMotIncrPeriodToVelGearRadS(input->actual_velocity);
}

double CanDriveTwitter::readTorqueNm()
{
    int rated_torque = 11; // 11 mNm

    return input->actual_torque * rated_torque / (1000 * 1000);
}

double CanDriveTwitter::readAnalogInput()
{
    return input->analog_input;
}

CanDriveTwitter::DriveState CanDriveTwitter::readDriveState()
{
    unsigned char status_lower = (unsigned char) input->status_word;
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
        if (bit5 && !bit6)
            return ST_READY_TO_SWITCH_ON;
    case 0x3:
        if (bit5 && !bit6)
            return ST_SWITCHED_ON;
    case 0x7:
        if (bit5 && !bit6)
            return ST_OPERATION_ENABLE;
        else if (!bit5 && !bit6)
            return ST_QUICK_STOP_ACTIVE;
    case 0x8:
        if (!bit6)
            return ST_FAULT;
    case 0xf:
        if (!bit6)
            return ST_FAULT_REACTION_ACTIVE;
    }

    std::cout << "Drive " << _drive_name << " in unknown state! Lower bit of status word: " << status_lower << std::endl;
}

bool CanDriveTwitter::isError()
{
    DriveState state = readDriveState();

    return (state == ST_FAULT_REACTION_ACTIVE) || (state == ST_FAULT);
}

unsigned int CanDriveTwitter::getError()
{
    unsigned char status_upper = (unsigned char) (input->status_word >> 8);

    return status_upper; 
}

bool CanDriveTwitter::requestEmergencyStop()
{
    uint16_t control_word = output->control_word;

    // enable quick stop
    control_word &= 0b111111101111011;
    control_word |= 0b000000000000010;

    output->control_word = control_word;

    int cnt = 100;

    DriveState state;

    do
    {
        usleep(10000); // sleep 0.01 s
        state = readDriveState();

        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::requestEmergencyStop: Could not emergency stop drive " << _drive_name << ". Last state was " << state << std::endl;
            return false;
        }
    }
    while (state != ST_QUICK_STOP_ACTIVE);

    return true;
}

void CanDriveTwitter::setDriveParam(DriveParam drive_param)
{
    _drive_param = drive_param;
}

DriveParam *CanDriveTwitter::getDriveParam()
{
    return &_drive_param;
}
