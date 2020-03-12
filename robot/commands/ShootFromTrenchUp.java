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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeMotor;
import frc.robot.subsystems.Launcher;

public class ShootFromTrenchUp extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndMove.
   */
  public ShootFromTrenchUp(Intake objIntake, IntakeMotor objIntakeMotor, Floor objFloor,
  Launcher objLauncher, DoubleSupplier dsSpeedIn, Feeder objFeederIn) {
    addCommands(

      new ParallelCommandGroup(
        new RollIn(objIntakeMotor),
        new RaiseIntake(objIntake),
        new LauncherSpeed(objLauncher, dsSpeedIn),
        new SequentialCommandGroup(
          new Wait().withTimeout(2.0),
          new FeedLauncherSlowly(objFeederIn)
        ),
        new SequentialCommandGroup(
          new Wait().withTimeout(3.0),
          new RaiseFloor(objFloor).withTimeout(1.0),
          new Wait().withTimeout(0.3),
          new RaiseFloor(objFloor).withTimeout(2.0)
        )
      )
    );

  }
}

