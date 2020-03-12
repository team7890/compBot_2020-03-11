/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

// Test Chassis Victors 
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// Test Chassis Speed Groups
// import edu.wpi.first.wpilibj.SpeedControllerGroup;

// Falcon Library Import
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
// NavX Import
import com.kauailabs.navx.frc.AHRS;

// This is the import from the Constants.java section also seen in code as the kCANIds.----
import frc.robot.Constants.kCANIds;
import frc.robot.Constants.kPneumatics;
import frc.robot.Constants.kRobotDims;

public class DriveTrain extends SubsystemBase {

  // Falcon Declarations
  // kCANIds are found in Constants.java section
  private final WPI_TalonFX leftPrimary = new WPI_TalonFX(kCANIds.iLeftPrimary);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(kCANIds.iLeftFollower);
  private final WPI_TalonFX rightPrimary = new WPI_TalonFX(kCANIds.iRightPrimary);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(kCANIds.iRightFollower);

  // //Test Chassis Declarations in place of Falcons
  // private final WPI_VictorSPX leftPrimary = new WPI_VictorSPX(1);
  // private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(2);
  // private final WPI_VictorSPX rightPrimary = new WPI_VictorSPX(3);
  // private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(4);
  private final DifferentialDrive objDiffDrive = new DifferentialDrive(leftPrimary, rightPrimary);

  private final Solenoid objSolenoidShift = new Solenoid(kCANIds.iPCM, kPneumatics.iShifter);

  // Test Chassis speed controller group config
  // private final SpeedController objScgLeftMotors = new SpeedControllerGroup(new WPI_VictorSPX(1), new WPI_VictorSPX(2));
  // private final SpeedController objScgRightMotors = new SpeedControllerGroup(new WPI_VictorSPX(3), new WPI_VictorSPX(4));
  // private final DifferentialDrive objDiffDrive = new DifferentialDrive(objScgLeftMotors, objScgRightMotors);

  // If you need to change the track dimentions, change it in the Constants.java section
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kRobotDims.dTrackWidth);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  Pose2d pose = new Pose2d();

  private double dRightDistance;  // cumulative distance in meters, since reset
  private double dLeftDistance;   // cumulative distance in meters, since reset
  private double dLeftCountsOld;  // cumulative encoder counts from previous code cycle, since reset
  private double dRightCountsOld; // cumulative encoder counts from previous code cycle, since reset

  public DriveTrain() {

    // Test Chassis leader/follower motor config
    // The followers will now follow the coding of the primaries
    leftFollower.follow(leftPrimary);
    rightFollower.follow(rightPrimary);
    // The commanded speeds for the falcons will now all be positive for forward motion -- the differential drive class handles inversion so set all to false here
    leftPrimary.setInverted(false);
    leftFollower.setInverted(false);
    rightPrimary.setInverted(false);
    rightFollower.setInverted(false);

    leftPrimary.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightPrimary.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    leftPrimary.configOpenloopRamp(0.30);
    leftFollower.configOpenloopRamp(0.00);
    rightPrimary.configOpenloopRamp(0.30);
    rightFollower.configOpenloopRamp(0.00);

    //starting distances
    dRightDistance = 0.0;
    dLeftDistance = 0.0;

  }

  @Override
  public void periodic() {
    // comment out for speed control group
    dLeftDistance = dLeftDistance + getLeftDistance(false, dLeftCountsOld);       // distance in meters
    dRightDistance = dRightDistance + getRightDistance(false, dRightCountsOld);   // distance in meters

    pose = odometry.update(getHeading(), dLeftDistance, dRightDistance);

    // // need to remember counts because distance = delta counts * gear ratio and we have shifting gearbox
    dLeftCountsOld = leftPrimary.getSelectedSensorPosition();
    dRightCountsOld = leftPrimary.getSelectedSensorPosition();
  }

  public void resetOdometry() {
    resetDistances();
    pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    odometry.resetPosition(pose, getHeading());
  }

  public void resetDistances() {
    leftPrimary.setSelectedSensorPosition(0);
    rightPrimary.setSelectedSensorPosition(0);
    dRightDistance = 0.0;
    dLeftDistance = 0.0;
  }

  public Rotation2d getHeading() {
    // return Rotation2d.fromDegrees(-gyro.getAngle());
    return Rotation2d.fromDegrees(-45.0);                 // without gyro, the above line crashes code
  }

  // public DifferentialDriveWheelSpeeds getSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(getSpeedGear("left", false), getSpeedGear("right", false)); // Need variable for the gear!!! Use the variable that refers to the piston position
  // }

  // private double getSpeedGear(String sSide, Boolean bHighGear) {
  //   if (bHighGear) {
  //     if (sSide.equals("left")) {
  //       // The sensor velocity is counts per 100 milliseconds so multiply by 10 to convert to seconds
  //       return leftPrimary.getSelectedSensorVelocity() / kRobotDims.dFalconConversionHigh;              //Double Check this please!!!
  //     }
  //     else {
  //       return rightPrimary.getSelectedSensorVelocity() / kRobotDims.dFalconConversionHigh; 
  //     }
  //   }
  //   else {
  //     if (sSide.equals("left")) {
  //       return leftPrimary.getSelectedSensorVelocity() / kRobotDims.dFalconConversionLow;               //Again please check this I may be wrong!!!
  //     }
  //     else {
  //       return rightPrimary.getSelectedSensorVelocity() / kRobotDims.dFalconConversionLow;
  //     }
  //   }
  // }

  public void shiftGear(boolean bGearIn) {
    objSolenoidShift.set(bGearIn);
  }

  private double getLeftDistance(boolean bHighGear, double dCountsIn) {
    // bHighGear is true for high gear (fast robot speed/low reduction ratio) and 
    if (bHighGear) {
      return (leftPrimary.getSelectedSensorPosition() - dCountsIn) / kRobotDims.dFalconConversionHigh;    // please check conversion!!!
    }
    else {
      return (leftPrimary.getSelectedSensorPosition() - dCountsIn) / kRobotDims.dFalconConversionLow;
    }
  }

  private double getRightDistance(boolean bHighGear, double dCountsIn) {
    // bHighGear is true for high gear (fast robot speed/low reduction ratio) and 
    if (bHighGear) {
      return (rightPrimary.getSelectedSensorPosition() - dCountsIn) / kRobotDims.dFalconConversionHigh;
    }
    else {
      return (rightPrimary.getSelectedSensorPosition() - dCountsIn) / kRobotDims.dFalconConversionLow;
    }
  }

  public void westCoastDrive(double dSpeed, double dTurn) {
    objDiffDrive.arcadeDrive(dSpeed, dTurn);
  }

  // Test Chassis Speed Groups
  // public void westCoastDrive(double dSpeed, double dTurn) {
  //   objDiffDrive.arcadeDrive(dSpeed, dTurn);
  // }

}
