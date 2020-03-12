/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Pneumatics imports
import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics extends SubsystemBase {

  private final Compressor objCompressor = new Compressor();

  public Pneumatics() {
    objCompressor.setClosedLoopControl(true);
  }

  @Override
  public void periodic() {
  }
  
}
