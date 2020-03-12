/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.kCANIds;
import frc.robot.Constants.kPneumatics;

public class Floor extends SubsystemBase {

  Solenoid objSolenoidFloor = new Solenoid(kCANIds.iPCM, kPneumatics.iRaiseFloor);

  public Floor() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveFloor(boolean bExtend) {
    objSolenoidFloor.set(bExtend);
  }
  
}
