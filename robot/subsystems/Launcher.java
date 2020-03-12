/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANIds;
import frc.robot.Constants.kSpeedMults;
import frc.robot.Constants.kLauncherPID;
import frc.robot.Constants.kFixedSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher extends SubsystemBase {

  CANSparkMax objNeo9 = new CANSparkMax(kCANIds.iLauncherOne, MotorType.kBrushless);
  CANSparkMax objNeo16 = new CANSparkMax(kCANIds.iLauncherTwo, MotorType.kBrushless);

  CANEncoder objEncoder9 = new CANEncoder(objNeo9);
  CANEncoder objEncoder16 = new CANEncoder(objNeo16);

  double dCmd9 = 0.0;
  double dCmd16 = 0.0;

  public Launcher() {

  }
  
  public void setSpeed(double dSpeedIn, boolean bPIDActive) {

    double dTarget;

    double dError9;
    double dDeriv9;
    double dErrorOld9;
    double dVelocity9;

    double dError16;
    double dDeriv16;
    double dErrorOld16;
    double dVelocity16;
    // if bPIDActive then input dSpeed is in RPM, otherwise in range -1 to +1

      // objNeo9.set(-kSpeedMults.dLauncherMult * dSpeed);
      // objNeo16.set(dSpeed);
      
      dVelocity9 = Math.abs(objEncoder9.getVelocity());
      dVelocity16 = objEncoder16.getVelocity();

      dTarget = dSpeedIn;

      dError9 = dTarget * kSpeedMults.dLauncherMult - dVelocity9;
      dError16 = dTarget - dVelocity16;

      dCmd9 = dCmd9 + dError9 * 0.04 / kFixedSpeeds.dLauncherMaxRPM;
      dCmd16 = dCmd16 + dError16 * 0.04 / kFixedSpeeds.dLauncherMaxRPM;

      if (dCmd9 < 0.0) {dCmd9 = 0.0;}
      if (dCmd16 < 0.0) {dCmd16 = 0.0;}

      objNeo9.set(-dCmd9);
      objNeo16.set(dCmd16);

      SmartDashboard.putNumber("PID Target", dTarget);
      SmartDashboard.putNumber("PID Command", dCmd9);
      SmartDashboard.putNumber("PID Error", dError9);
      SmartDashboard.putNumber("PID Velocity", dVelocity9);
      SmartDashboard.putNumber("PID Command16", dCmd16);
      SmartDashboard.putNumber("PID Error16", dError16);
      SmartDashboard.putNumber("PID Velocity16", dVelocity16);
  }


  public double getVelocity9() {
    return objEncoder9.getVelocity();
  }

  public void stop() {
    objNeo9.stopMotor();
    objNeo16.stopMotor();
  }

  public void togglePID(boolean bPIDActive) {
    // TODO: So maybe try out a backup system if the PID decides to not work... !!!
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
