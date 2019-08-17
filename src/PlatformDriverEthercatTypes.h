#pragma once

#include <memory>
#include <string>

namespace platform_driver_ethercat
{
class CanDriveTwitter;

struct DriveConfig
{
    int iEncIncrPerRevMot;
    double dGearRatio;
    double dBeltRatio;
    int iSign;
    double dPosLimitLowIncr;
    double dPosLimitHighIncr;
    double dVelMaxEncIncrS;
    double dPtpVelDefaultIncrS;
    double dAccIncrS2;
    double dDecIncrS2;
    bool bIsSteer;
    double dCurrentToTorque;
    double dCurrMax;
    int iEncOffsetIncr;
    double dAnalogFactor;
    double dNominalCurrent;
};
}
