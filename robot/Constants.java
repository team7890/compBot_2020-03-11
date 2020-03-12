/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class kCANIds {
        public static final int iLeftPrimary = 1;
        public static final int iLeftFollower = 2;
        public static final int iRightPrimary = 3;
        public static final int iRightFollower = 4;
        public static final int iClimbLeft = 5;
        public static final int iClimbRight = 6;
        public static final int iTraveler = 7;
        public static final int iFeed = 8;
        public static final int iLauncherOne = 9;
        public static final int iIndexer = 10;
        public static final int iIntake = 11;
        public static final int iLaunchAngle = 12;
        public static final int iColorWheel = 13;
        public static final int iElevator = 14;
        public static final int iLauncherTwo = 16;
        public static final int iPCM = 17;
    }

    public static final class kRobotDims {
        public static final double dTrackWidth = Units.inchesToMeters(24.75);           // track width in meters
        public static final double dWheelDia = Units.inchesToMeters(6.0);               // wheel diameter in meters
        public static final double dGearBoxRatioHigh = 5.133;
        public static final double dGearBoxRatioLow = 15.0;
        public static final double dFalconCountsPerRev = 2048.0;                         // TODO: NEED TO CHECK!!!
        public static final double dFalconEncoderToVelocity = 10.0 / dFalconCountsPerRev * Math.PI * Units.inchesToMeters(dWheelDia);
        public static final double dFalconConversionHigh = dFalconEncoderToVelocity / dGearBoxRatioHigh;
        public static final double dFalconConversionLow = dFalconEncoderToVelocity / dGearBoxRatioLow;
    }

    public static final class kPneumatics {
        public static final int iShifter = 0;       // works correctly
        public static final int iRaiseFloor = 1;    // works correctly
        public static final int iIntake = 2;        // works correctly
        public static final int iColorWheel = 4;    // not implemented (no solenoid yet)
        public static final int iIndexer = 3;       // not implemented (not tested)
    }

    public static final class kSpeedMults {
        // subsystem multipliers
        public static final double dIntakeMult = 0.6;
        public static final double dElevatorMult = 0.2;
        public static final double dTilterMult = 0.2;
        public static final double dLauncherMult = 0.75;
    }

    public static final class kFixedSpeeds {
        // subsystem speed constants
        public static final double dIntake = 0.90;

        // automated speeds
        public static final double dAutoLaunchDown = 3700.0;
        public static final double dTrenchLaunchUp = 2800.0;
        public static final double dTrenchLaunchDown = 2900.0;

        // command Speed constants
        public static final double dFeedWheel = 1.0;
        public static final double dIndexer = 0.5;
        public static final double dLauncherMaxRPM = 5676.0;
        public static final double dLauncherLowRPM = 2000.0;
        public static final double dLauncherInitLineRPM = 4500.0;
        public static final double dWinchSpeed = 0.5;
    }

    public static final class kLauncherPID {
        public static final double dKp = 0.000;
        public static final double dKi = 0.000005;
        public static final double dKd = 0.0;
        public static final double dKiz = 0.0;
        public static final double dKff = 0.0;
        public static final double dMaxOutput = 1.0;
        public static final double dMinOutput = -1.0;
    }

    public static final class kTilter {
        public static final double dMaxEncoder = 35.2;
        public static final double dMinEncoder = 0.0;
    }
}
