/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Launcher;

public class ShootHighAtGoal extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndMove.
   */
  public ShootHighAtGoal (Launcher objLauncher, DoubleSupplier dsSpeedIn, Feeder objFeederIn,
  DoubleSupplier dSpeedIn, DoubleSupplier dTurnIn, DriveTrain objDriveTrain, Floor objFloor, DoubleSupplier dsSlowDownIn) {
    addCommands(
      new ParallelCommandGroup(
        new LauncherSpeed(objLauncher, dsSpeedIn),
        new DriveRobot((dSpeedIn), dTurnIn, objDriveTrain).withTimeout(0.2),
        new SequentialCommandGroup(
          new FeedLauncherSlowly(objFeederIn),
          new Wait().withTimeout(1.5),
          new RaiseFloor(objFloor)
        )
      ).withTimeout(10.0)
    );
  }
}
