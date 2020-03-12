/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Launcher;

public class AutoLaunchAndMove extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndMove.
   */
  public AutoLaunchAndMove(Floor objFloor, Launcher objLauncher, DoubleSupplier dsSpeedIn, Feeder objFeederIn,
  DoubleSupplier dSpeedIn, DoubleSupplier dTurnIn, DriveTrain objDriveTrain) {
    addCommands(
      new ParallelCommandGroup(
        new RaiseFloor(objFloor),
        new LauncherSpeed(objLauncher, dsSpeedIn),
        new SequentialCommandGroup(
          new Wait().withTimeout(1.0),
          new FeedLauncherSlowly(objFeederIn)

        )
      ).withTimeout(2.5),
      new DriveRobot(dSpeedIn, dTurnIn, objDriveTrain).withTimeout(2.0)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
 
}
