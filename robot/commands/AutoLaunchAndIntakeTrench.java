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

public class AutoLaunchAndIntakeTrench extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndMove.
   */
  public AutoLaunchAndIntakeTrench(Floor objFloor, Launcher objLauncher, DoubleSupplier dsSpeedIn, Feeder objFeederIn,
  DoubleSupplier dSpeedIn, DoubleSupplier dTurnIn, DriveTrain objDriveTrain, Intake objIntake, IntakeMotor objIntakeMotor) {

    DoubleSupplier dTurnLeft = () -> {return -1.0;};
    DoubleSupplier dTurnRight = () -> {return 1.0;};
    DoubleSupplier dSpeedStop = () -> {return 0.0;};
    DoubleSupplier dSpeedSpecific = () -> {return -0.5;};
    DoubleSupplier dSpeedFast = () -> {return 0.75;};

    addCommands(


      new ParallelCommandGroup(
        new RaiseFloor(objFloor),
        new LauncherSpeed(objLauncher, dsSpeedIn),
        new SequentialCommandGroup(
          new Wait().withTimeout(6.0),
          new FeedLauncherSlowly(objFeederIn)

        )
      ).withTimeout(10.0),
      new DriveRobot(dSpeedIn, dTurnIn, objDriveTrain).withTimeout(2.0)


      // new SequentialCommandGroup(
      //   new ParallelCommandGroup(
      //     new RaiseFloor(objFloor),
      //     new LauncherSpeed(objLauncher, dsSpeedIn),
      //     new SequentialCommandGroup(
      //       new Wait().withTimeout(1.0),
      //       new FeedLauncherSlowly(objFeederIn).withTimeout(5.0)
      //     )
      //   ).withTimeout(2.5),
      //   new DriveRobot(dSpeedSpecific, dTurnIn, objDriveTrain, dsSlowDownIn).withTimeout(1.0)
        // new RaiseIntake(objIntake),
        // new DriveRobot(dSpeedFast, dTurnIn, objDriveTrain, dsSlowDownIn).withTimeout(0.9),
        // new RollIn(objIntakeMotor),
        // new DriveRobot(dSpeedStop, dTurnRight, objDriveTrain, dsSlowDownIn).withTimeout(1.5)
        // new DriveRobot(dSpeedSpecific, dTurnRight, objDriveTrain, dsSlowDownIn).withTimeout(1.75)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
 
}

