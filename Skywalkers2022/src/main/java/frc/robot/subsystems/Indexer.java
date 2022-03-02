// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  CANSparkMax indexerMotor = new CANSparkMax(IndexerConstants.kMotorPort, MotorType.kBrushless);
  // implement color sensor
  // implement beam breaker sensors

  public Indexer() {

    indexerMotor.restoreFactoryDefaults();
    indexerMotor.setInverted(IndexerConstants.kInvert);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutput(double speed) {
    indexerMotor.set(speed);
  }

  public void getColor() {
    // return color that color sensor detects
  }

  public void isBallAtEntry() {
    // return true if bottom beam breaker is broken
  }
  
  public void isBallAtExit() {
    // return true if top beam breaker is broken
  }

}
