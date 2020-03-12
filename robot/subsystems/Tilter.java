/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.kCANIds;
import frc.robot.Constants.kSpeedMults;
import frc.robot.Constants.kTilter;

public class Tilter extends SubsystemBase {
  private final CANSparkMax TilterMotor; 
  private final CANEncoder tilterEncoder;
  
  /**
   * Creates a new Tilter.
   */
  public Tilter() {
    TilterMotor = new CANSparkMax(kCANIds.iLaunchAngle, MotorType.kBrushless);
    tilterEncoder = new CANEncoder(TilterMotor);
    tilterEncoder.setPosition(0.0);
  }

  // public void set(DoubleSupplier dSpeed) {
  //   if (tilterEncoder.getPosition() < kTilter.dMaxEncoder & dSpeed.getAsDouble() < 0.0) {
  //     TilterMotor.set(-kSpeedMults.dTilterMult * dSpeed.getAsDouble());
  //   }
  //   else {
  //     if (tilterEncoder.getPosition() > kTilter.dMinEncoder & dSpeed.getAsDouble() > 0.0) {
  //       TilterMotor.set(-kSpeedMults.dTilterMult * dSpeed.getAsDouble());
  //     }
  //     else {
  //       TilterMotor.set(0.0);
  //     }
  //   }
  // }

  public void set(double dSpeed) {
    if (tilterEncoder.getPosition() < kTilter.dMaxEncoder & dSpeed < 0.0) {
      TilterMotor.set(-kSpeedMults.dTilterMult * dSpeed);
    }
    else {
      if (tilterEncoder.getPosition() > kTilter.dMinEncoder & dSpeed > 0.0) {
        TilterMotor.set(-kSpeedMults.dTilterMult * dSpeed);
      }
      else {
        TilterMotor.set(0.0);
      }
    }
  }

  public double sendEncoderTicks() {
    return tilterEncoder.getPosition();
  }

  public void setStop() {
    TilterMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}
