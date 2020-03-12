/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANIds;
import frc.robot.Constants.kFixedSpeeds;

public class IntakeMotor extends SubsystemBase {
  
  VictorSPX objIntakeMotor = new VictorSPX(kCANIds.iIntake);

  public IntakeMotor() {

  }
  
  public void rollIn() {
    objIntakeMotor.set(ControlMode.PercentOutput, -kFixedSpeeds.dIntake); // TODO: May want to change the speed !!!
  }

  public void rollOut() {
    objIntakeMotor.set(ControlMode.PercentOutput, kFixedSpeeds.dIntake); // TODO: May want to change the speed !!!
  }

  public void stop() {
    objIntakeMotor.set(ControlMode.PercentOutput, 0.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
