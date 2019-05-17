/****************************************************************
 *
 * Copyright (c) 2013
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: ROBS
 * stack name: MotionControl
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Martin Azkarate, email:martin.azkarate@esa.int
 * Supervised by: Pantelis Poulakis, email:pantelis.poulakis@esa.int
 *
 * Date of creation: Jan 2013
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_canopen_motor
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2010
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#ifndef DRIVEPARAM_INCLUDEDEF_H
#define DRIVEPARAM_INCLUDEDEF_H


#include <math.h>
const double dPI = 4*atan(1.0);

/**
 * This class contains specific Parameters of a given motor drive system.
 * It also provides conversion functionality from user units to motor units (and vice versa).
 */
class DriveParam
{

private:

    int m_iEncIncrPerRevMot;                /**< Encoder increments per revolution motor shaft */
    double m_dBeltRatio;                    /**< Belt ratio of the drive system if it has any, else 1 */
    double m_dGearRatio;                    /**< Total Gear ratio of the drive system */
    int m_iSign;                            /**< Direction of positive motion, 1 or -1 */
    double m_dPosLimitLowIncr;              /**< Min position limit, encoder increments */
    double m_dPosLimitHighIncr;             /**< Max position limit, encoder increments */
    double m_dVelMaxEncIncrS;               /**< Max velocity limit, encoder increments per second */
    double m_dPtpVelDefaultIncrS;               /**< Default velocity for Point to Point commands, encoder increments per second */
    double m_dAccIncrS2;                    /**< Max acceleration limit, encoder increments per second2 */
    double m_dDecIncrS2;                    /**< Max deceleration limit, encoder increments per second2 */
    double m_dPosGearRadToPosMotIncr;       /**< Conversion factor for motor position, from radians to encoder increments */
    double m_dVelGearRadSToVelMotIncrS;     /**< Conversion factor for motor velocity, from radians/s to encoder increments/s */
    bool m_bIsSteer;                        /**< Flag to select control mode, position or velocity controlled */
    double m_dCurrToTorque;                 /**< Conversion factor for motor active current [A] into torque [Nm] */
    double m_dCurrMax;                      /**< Max allowed current */
    int m_iEncOffsetIncr;                   /**< Encoder offset for absolute zero position */
    int m_iHomingDigIn;                     /**< specifies which digital input is used for homing signal, in case is used */
    double m_dAnalogFactor;                 /**< Factor to convert the Analog input internal object value into physical units (Volts). Same as WS[91] */
    double m_dNominalCurrent;               /**< Nominal current of the motor. Used as Factor to convert the Current reading internal object value into physical units (mAmps). Same as CL[1] */

public:

    /**
     * Default constructor.
     */
    DriveParam()
    {
        m_bIsSteer = true; /// default value
    }

    /**
     * Sets the drive parameters.
     * @param iEncIncrPerRevMot encoder increments per revolution of the motor shaft
     * @param dBeltRatio ratio of the gear
     * @param dGearRatio ratio of the gear
     * @param iSign change -1 for changing the motion direction
     * @param dPosLimitLowIncr low limit for the position command
     * @param dPosLimitHighIncr high limit for the position command
     * @param dVelMaxEncIncrS maximum velocity given in encoder increments per second
     * @param dAccIncrS2 acceleration in encoder increments per s^2
     * @param dDecIncrS2 deceleration in encoder increments per s^2
     * @param bIsSteer signal whether the motor controlled in position or in velocity
     * @param dCurrToTorque conversion factor for motor active current into torque
     * @param dCurrMax Max allowed current
     * @param iEncOffsetIncr Encoder offset for absolute zero position
     */
    void setParam(
        int iEncIncrPerRevMot,
        double dBeltRatio,
        double dGearRatio,
        int iSign,
        double dPosLimitLowIncr,
        double dPosLimitHighIncr,
        double dVelMaxEncIncrS,
        double dPtpVelDefaultIncrS,
        double dAccIncrS2,
        double dDecIncrS2,
        bool bIsSteer,
        double dCurrToTorque,
        double dCurrMax,
        int iEncOffsetIncr,
        double dAnalogFactor,
        double dNominalCurrent)
    {
        m_iEncIncrPerRevMot = iEncIncrPerRevMot;
        m_dBeltRatio = dBeltRatio;
        m_dGearRatio = dGearRatio;
        m_iSign = iSign;
        m_dPosLimitLowIncr = dPosLimitLowIncr;
        m_dPosLimitHighIncr = dPosLimitHighIncr;
        m_dVelMaxEncIncrS = dVelMaxEncIncrS;
        m_dPtpVelDefaultIncrS = dPtpVelDefaultIncrS;
        m_dAccIncrS2 = dAccIncrS2;
        m_dDecIncrS2 = dDecIncrS2;
        m_dPosGearRadToPosMotIncr = m_iEncIncrPerRevMot * m_dGearRatio * m_dBeltRatio / (2. * dPI);
        m_dVelGearRadSToVelMotIncrS = m_iEncIncrPerRevMot * m_dGearRatio * m_dBeltRatio / (2. * dPI);
        m_bIsSteer = bIsSteer;
        m_dCurrToTorque = dCurrToTorque;
        m_dCurrMax = dCurrMax;
        m_iEncOffsetIncr = iEncOffsetIncr;
        m_iHomingDigIn = 0;
        m_dAnalogFactor = dAnalogFactor;
        m_dNominalCurrent = dNominalCurrent;
    }

    //* Overloaded Method
    void setParam(
        int iEncIncrPerRevMot,
        double dBeltRatio,
        double dGearRatio,
        int iSign,
        double dVelMaxEncIncrS,
        double dAccIncrS2,
        double dDecIncrS2)
    {
        m_iEncIncrPerRevMot = iEncIncrPerRevMot;
        m_dBeltRatio = dBeltRatio;
        m_dGearRatio = dGearRatio;
        m_iSign = iSign;
        m_dVelMaxEncIncrS = dVelMaxEncIncrS;
        m_dAccIncrS2 = dAccIncrS2;
        m_dDecIncrS2 = dDecIncrS2;
        m_iHomingDigIn = 0;
        m_dPosGearRadToPosMotIncr = m_iEncIncrPerRevMot * m_dGearRatio * m_dBeltRatio / (2. * dPI);
    }

    //* Overloaded Method
    void setParam(
        int iEncIncrPerRevMot,
        double dBeltRatio,
        double dGearRatio,
        int iSign,
        double dVelMaxEncIncrS,
        double dAccIncrS2,
        double dDecIncrS2,
        int iEncOffsetIncr,
        bool bIsSteer,
        double dCurrToTorque,
        double dCurrMax,
        int iHomingDigIn)
    {
        m_iEncIncrPerRevMot = iEncIncrPerRevMot;
        m_dBeltRatio = dBeltRatio;
        m_dGearRatio = dGearRatio;
        m_iSign = iSign;
        m_dVelMaxEncIncrS = dVelMaxEncIncrS;
        m_dAccIncrS2 = dAccIncrS2;
        m_dDecIncrS2 = dDecIncrS2;
        m_iEncOffsetIncr = iEncOffsetIncr;
        m_bIsSteer = bIsSteer;
        m_dPosGearRadToPosMotIncr = m_iEncIncrPerRevMot * m_dGearRatio * m_dBeltRatio / (2. * dPI);
        m_dCurrToTorque = dCurrToTorque;
        m_dCurrMax = dCurrMax;
        m_iHomingDigIn = iHomingDigIn;
    }



    /**
     * Converts position and velocity.
     * @param dPosRad position in radian
     * @param dVelRadS velocity in radians per second
     * @param piPosIncr converted position in increments
     * @param piVelIncrPeriod converted velocity in increments of period
     */
    void PosVelRadToIncr(double dPosRad, double dVelRadS, int* piPosIncr, int* piVelIncrPeriod)
    {
        *piPosIncr = PosGearRadToPosMotIncr(dPosRad);
        *piVelIncrPeriod = VelGearRadSToVelMotIncrPeriod(dVelRadS);
    }

    /**
     * Converts the temperature in degree Celsius.
     * The temperature measure is only supported for the drive neo.
     * @param iTempIncr temperature in a special internal unit
     */
    int TempMeasIncrToGradCel(int iTempIncr)
    {
        double dTempMeasGradCel;

        dTempMeasGradCel = 0.0002 * (iTempIncr * iTempIncr) - 0.2592 * iTempIncr + 105;

        return (int)dTempMeasGradCel;
    }

    /**
     * Converts revolution angle form radians to encoder increments.
     * @param dPosGearRad angle in radians
     */
    int PosGearRadToPosMotIncr(double dPosGearRad)
    {
        return ((int)(dPosGearRad * m_dPosGearRadToPosMotIncr));
    }

    /**
     * Conversions of encoder increments to gear position in radians.
     * @param iPosIncr encoder position in increments
     */
    double PosMotIncrToPosGearRad(int iPosIncr)
    {
        return ((double)iPosIncr / m_dPosGearRadToPosMotIncr);
    }

    /**
     * Conversions of gear velocity in rad/s to encoder increments per measurement period.
     * @param dVelGearRadS velocity in radians per second
     */
    int VelGearRadSToVelMotIncrPeriod(double dVelGearRadS)
    {
        return ((int)(dVelGearRadS * m_dVelGearRadSToVelMotIncrS));
    }

    /**
     * Conversions of  encoder increments per measurement period to gear velocity in rad/s.
     * @param iVelMotIncrPeriod encoder velocity in increments per second
     */
    double VelMotIncrPeriodToVelGearRadS(int iVelMotIncrPeriod)
    {
        return ((double)iVelMotIncrPeriod / m_dVelGearRadSToVelMotIncrS);
    }


    /**
     * Returns the sign for the motion direction.
     */
    int getSign()
    {
        return m_iSign;
    }

    /**
     * Gets the maximum velocity of the drive in increments per second.
     */
    double getVelMax()
    {
        return m_dVelMaxEncIncrS;
    }

    /**
     * Gets the maximum position of the drive in increments.
     */
    double getPosMax()
    {
        return m_dPosLimitHighIncr;
    }

    /**
     * Gets the minimum position of the drive in increments.
     */
    double getPosMin()
    {
        return m_dPosLimitLowIncr;
    }

    /**
     * Set the maximum acceleration.
     * @param dMaxAcc Maximum acceleration
     */
    void setMaxAcc(double dMaxAcc)
    {
        m_dAccIncrS2 = dMaxAcc;
    }

    /**
     * Get the maximum acceleration.
     * @return Maximum acceleration
     */
    double getMaxAcc()
    {
        return m_dAccIncrS2;
    }

    /**
     * Set the maximum deceleration.
     * @param dMaxAcc Maximum deceleration
     */
    void setMaxDec(double dMaxDec)
    {
        m_dDecIncrS2 = dMaxDec;
    }

    /**
     * Get the maximum deceleration.
     * @return Maximum deceleration
     */
    double getMaxDec()
    {
        return m_dDecIncrS2;
    }

    /**
     * Set the maximum velocity.
     * @param dMaxVel Maximum velocity
     */
    void setMaxVel(double dMaxVel)
    {
        m_dVelMaxEncIncrS = dMaxVel;
    }

    /**
     * Get the maximum velocity in increments per second.
     * @return Maximum velocity [inc/sec].
     */
    double getMaxVel()
    {
        return m_dVelMaxEncIncrS;
    }

    /**
     * Get the Point to Point default velocity in increments per second.
     * @return velocity [inc/sec].
     */
    double getPtpVelDefault()
    {
        return m_dPtpVelDefaultIncrS;
    }

    /**
     * Set the Point to Point default velocity in increments per second.
     * @param dPtpVelDef Default velocity [inc/sec].
     */
    void setPtpVelDefault(double dPtpVelDef)
    {
        m_dPtpVelDefaultIncrS=dPtpVelDef;
    }

    /**
     * Get the Analog factor value.
     * @return factor [no units].
     */
    double getAnalogFactor()
    {
        return m_dAnalogFactor;
    }

    /**
     * Set the Analog factor value.
     * @param dAnalogFactor Analog factor [no units].
     */
    void setAnalogFactor(double dAnalogFactor)
    {
        m_dAnalogFactor=dAnalogFactor;
    }

    /**
     * Get the Nominal current value.
     * @return Nominal current [Amps].
     */
    double getNominalCurrent()
    {
        return m_dNominalCurrent;
    }

    /**
     * Set the Nominal Current value.
     * @param dNominalCurrent Nominal current [Amps].
     */
    void setNominalCurrent(double dNominalCurrent)
    {
        m_dNominalCurrent=dNominalCurrent;
    }

    /**
     * Get the gear ratio.
     * @return The gear ratio.
     */
    double getGearRatio()
    {
        return m_dGearRatio;
    }
    /**
     * Get the belt ratio.
     * @return The belt ratio.
     */
    double getBeltRatio()
    {
        return m_dBeltRatio;
    }

    /**
     * Get the EncoderOffset
     * @return the Encoderoffset
     */
    int getEncOffset()
    {
        return m_iEncOffsetIncr;
    }

    /**
     * Get the DriveType - If it's a Steering or Driving Motor
     * @return the Encoderoffset
     */
    bool getIsSteer()
    {
        return m_bIsSteer;
    }

//    /**
//     * Sets the control mode - if it's position or velocity control
//     */
//    bool setIsSteer(bool isPositionControl)
//    {
//        m_bIsSteer = isPositionControl;
//    }

    /**
     * Get the DriveType - If it's a Steering or Driving Motor
     * @return the Encoderoffset
     */
    int getEncIncrPerRevMot()
    {
        return m_iEncIncrPerRevMot;
    }
    /**
     * Get factor to convert motor active current [A] into torque [Nm]
     */
    double getCurrToTorque()
    {
        return m_dCurrToTorque;
    }
    /**
     * Get maximum current allowed
     */
    double getCurrMax()
    {
        return m_dCurrMax;
    }
    /**
     * Get digital Input for Homing signal
     */
    int getHomingDigIn()
    {
        return m_iHomingDigIn;
    }
    /**
     * Set digital Input for Homing signal
     */
    void setHomingDigIn(int HomingDigIn)
    {
        m_iHomingDigIn = HomingDigIn;
    }
};
#endif
