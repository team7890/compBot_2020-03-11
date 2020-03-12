/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class CenterRobotInPlace extends CommandBase {
  DriveTrain objDriveTrain;
  /**
   * Creates a new CenterRobotInPlace.
   */
  public CenterRobotInPlace(DriveTrain objDriveTrainIn) {
    objDriveTrain = objDriveTrainIn;
    addRequirements(objDriveTrain);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    double Kp = 0.00008;
    double dMinCommand = 0.05;

    double dtx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    boolean btv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);

    double dHeadingError = -dtx;
    double dSteeringCmd = 0.0;

    if (dtx > 1.0) {
      dSteeringCmd = Kp * dHeadingError - dMinCommand;
    } else if (dtx < -1.0) {
      dSteeringCmd = Kp * dHeadingError + dMinCommand;
    }

    if (!btv) {
      SmartDashboard.putString("Targeting Status", "No Target");
      dSteeringCmd = 0.0;
    }
    else if (Math.abs(dtx) < 1.1) {
      SmartDashboard.putString("Targeting Status", "Finished Target");
      dSteeringCmd = 0.0;
    } 
    else {
      SmartDashboard.putString("Targeting Status", "Driving To Target");
    }  

    objDriveTrain.westCoastDrive(0.0, dSteeringCmd);    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
