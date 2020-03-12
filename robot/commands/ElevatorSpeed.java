/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSpeed extends CommandBase {

  double dEncoderTicks;
  Elevator objElevator;
  private DoubleSupplier dsSpeed;

  public ElevatorSpeed(Elevator objElevatorIn, DoubleSupplier speedSupplierIn) {
    objElevator = objElevatorIn;
    dsSpeed = speedSupplierIn;
    addRequirements(objElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dEncoderTicks = objElevator.sendEncoderTicks();
    objElevator.setSpeed(-dsSpeed.getAsDouble());
    SmartDashboard.putNumber("Elevator Cmd", -dsSpeed.getAsDouble());
    SmartDashboard.putNumber("Elevator Encoder", dEncoderTicks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objElevator.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
