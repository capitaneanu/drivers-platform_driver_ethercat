#include "CanDriveTwitter.h"

CanDriveTwitter::CanDriveTwitter(CanOverEthercat *can_interface, std::strint device_name)
: _can_interface(can_interface), _device_name(device_name)
{
}

CanDriveTwitter::~CanDriveTwitter()
{
}

bool CanDriveTwitter::init()
{
	std::cout << "CanDriveTwitter::init: Start init for drive " << m_sName << std::endl;

    return start();
}

bool CanDriveTwitter::startup()
{
    State state = getState();
    int cnt = 100;

    while (state != ST_OPERATION_ENABLE)
    {
        switch (state)
        {
        case ST_FAULT:
            output->control_word = 0x0080; // fault reset
            break;
        case ST_QUICK_STOP:
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
        }
        
        usleep(100000); // sleep 0.1 s
        state = getState();

        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::startup: Could not start up drive " << m_sName << ". Last state was " << state << std::endl;
            return false;
        }
    }

	std::cout << "CanDriveTwitter::startup: Drive " << m_sName << " started up." << std::endl;
	return true;
}

bool CanDriveTwitter::shutdown()
{	
    State state = getState();
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
        case ST_QUICK_STOP:
            output->control_word = 0x0004; // disable quick stop & disable voltage
            break; 
        }
        
        usleep(100000); // sleep 0.1 s
        state = getState();

        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::shutdown: Could not shut down drive " << m_sName << ". Last state was " << state << std::endl;
            return false;
        }
    }

	std::cout << "CanDriveTwitter::shutdown: Drive " << m_sName << " shut down." << std::endl;
	return true;
}

bool CanDriveTwitter::reset()
{
	return stop() && start();
}

CanDriveTwitter::OperationMode CanDriveTwitter::getOperationMode()
{
    return (OperationMode) input->operation_mode_display;
}

bool CanDriveTwitter::setOperationMode(CanDriveTwitter::OperationMode mode)
{
    output->operation_mode = mode; 

    int cnt = 1000;

    while (getOperationMode() != mode)
    {
        usleep(1000); // sleep 0.001 s

        if (cnt-- == 0)
        {
	        std::cout << "CanDriveTwitter::setOperationMode: Could not set operation mode of drive " << m_sName << ". Requested mode was " << mode << std::endl;
            return false;
        }
    }

    return true;
}

void CanDriveTwitter::positionCommandRad(double dPosGearRad, double dVelGearRadS)
{
}

void CanDriveTwitter::positionSetPointRad(double dPosGearRad, double dVelGearRadS)
{
}

void CanDriveTwitter::velocityCommandRadS(double dVelGearRadS)
{
}

void CanDriveTwitter::velocitySetPointRadS(double dVelGearRadS)
{
}

void CanDriveTwitter::torqueCommandNm(double dTorqueNm)
{
}

void CanDriveTwitter::commandSetPoint()
{
}

bool CanDriveTwitter::checkTargetReached()
{
    unsigned char bit10 = (unsigned char) ((input->status_word >> 10) & 0x0001);

    return (bool) bit10;
}

double CanDriveTwitter::getPositionRad()
{
}

double CanDriveTwitter::getVelocityRadS()
{
}

double CanDriveTwitter::getTorqueNm()
{
}

double CanDriveTwitter::getAnalogInput()
{
}

CanDriveTwitter::StateTwitter CanDriveTwitter::getState()
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

    std::cout << "Drive " << _device_name << " in unknown state! Lower bit of status word: " << status_lower << std::endl;
}

bool CanDriveTwitter::isError()
{
    StateTwitter state = getState();

    return (state == ST_FAULT_REACTION_ACTIVE) || (state == ST_FAULT);
}

unsigned int CanDriveTwitter::getError()
{
    unsigned char status_upper = (unsigned char) (input->status_word >> 8);

    return status_upper; 
}

bool CanDriveTwitter::setEmergencyStop()
{
}

bool CanDriveTwitter::resetEmergencyStop()
{
}


void CanDriveTwitter::setDriveParam(DriveParam driveParam)
{
}

DriveParam *CanDriveTwitter::getDriveParam()
{
}

p
