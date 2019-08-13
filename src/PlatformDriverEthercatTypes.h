#pragma once

#include <string>
#include <vector>

namespace platform_driver_ethercat
{
    struct GenericSlaveParams
    {
        unsigned int slave_id;
        std::string name;
    };

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

    struct DriveSlaveParams : public GenericSlaveParams
    {
        DriveConfig config;
        bool enabled;
        bool has_temp_sensor;
    };

    struct FtsSlaveParams : public GenericSlaveParams {};

	typedef std::vector<GenericSlaveParams> GenericSlaveMapping;
	typedef std::vector<DriveSlaveParams> DriveSlaveMapping;
	typedef std::vector<FtsSlaveParams> FtsSlaveMapping;
}
