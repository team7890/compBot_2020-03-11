/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Tilter;

public class TiltLauncher extends CommandBase {

  Tilter objTilter;
  DoubleSupplier dsTiltCommand;
  boolean bShootLow;
  boolean bShootInitLine;
  boolean bShootHighAtGoal;
  boolean bShoot;
  boolean bShootOld;
  double dEncoderTicks;
  final double dTargetHighAtGoal = 10.0;
  final double dTargetLow = 2.0;
  final double dTargetInitLine = 0.0;
  double dTarget;
  double dTiltCmd;
  double dError;
  double dErrorOld;
  double dDeriv;

  public TiltLauncher(Tilter objTilterIn, DoubleSupplier dsTiltCommandIn) {
    objTilter = objTilterIn;
    dsTiltCommand = dsTiltCommandIn;
    addRequirements(objTilter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bShootLow = SmartDashboard.getBoolean("Low Shot", false);
    bShootInitLine = SmartDashboard.getBoolean("Init Line Shot", false);
    bShootHighAtGoal = SmartDashboard.getBoolean("High at Goal", false);
    dEncoderTicks = objTilter.sendEncoderTicks();
    bShoot = bShootLow || bShootInitLine || bShootHighAtGoal;
    if (bShootLow) {dTarget = dTargetLow;}
    if (bShootInitLine) {dTarget = dTargetInitLine;}
    if (bShootHighAtGoal) {dTarget = dTargetHighAtGoal;}

    if (bShoot & !bShootOld) {
      dErrorOld = 0.0;
    }
    if (bShoot) {
      dError = dEncoderTicks - dTarget;
      dDeriv = dError - dErrorOld;
      dTiltCmd = dTiltCmd + 0.0001 * dError + 0.0003 * dDeriv;
      if (Math.abs(dError) < 0.07) {
        dTiltCmd = -0.075;
      }
      objTilter.set(dTiltCmd);
    }
    else {
      dTiltCmd = dsTiltCommand.getAsDouble();
      objTilter.set(dTiltCmd);
    }
    SmartDashboard.putNumber("Tilt Encoder", dEncoderTicks);
    SmartDashboard.putNumber("Tilt Cmd", dTiltCmd);
    bShootOld = bShootLow;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objTilter.setStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
