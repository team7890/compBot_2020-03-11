/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveRobot extends CommandBase {

  private final DriveTrain objDriveTrain;
  private DoubleSupplier dsSpeed;
  private DoubleSupplier dsTurn;
  private DoubleSupplier dSlowDown;

  public DriveRobot(DoubleSupplier dSpeedIn, DoubleSupplier dTurnIn, DriveTrain objDriveTrainIn) {
    objDriveTrain = objDriveTrainIn;
    dsSpeed = dSpeedIn;
    dsTurn = dTurnIn;
    addRequirements(objDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objDriveTrain.westCoastDrive(dsSpeed.getAsDouble(), dsTurn.getAsDouble() / 2.0);
    SmartDashboard.putNumber("Drive Speed", dsSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objDriveTrain.westCoastDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
