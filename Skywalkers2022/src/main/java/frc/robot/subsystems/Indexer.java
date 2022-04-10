// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax indexerMotor = new CANSparkMax(IndexerConstants.kMotorPort, MotorType.kBrushless);
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  public Indexer() {
    indexerMotor.restoreFactoryDefaults();
    indexerMotor.setInverted(IndexerConstants.kInvert);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    // SmartDashboard.putNumber("Indexer Voltage", indexerMotor.get());
  }

  public void setOutput(double speed) {
    if (Math.abs(speed) < 0.05) {
      speed = 0;
    }
    indexerMotor.set(speed);
  }

  public void on() {
    setOutput(IndexerConstants.kIndexerSpeed);
  }

  public void off() {
    setOutput(0);
  }

  public void moveUntilBall(double speed) {
    double proximity = colorSensor.getProximity();
    if (proximity < 300) {
      setOutput(speed);
    } else {
      off();
    }
  }

  public void moveUntilNoBall(double speed) {
    double proximity = colorSensor.getProximity();
    if (proximity > 300) {
      setOutput(speed);
    } else {
      off();
    }
  }
  
  public boolean isBallAtExit() {
    double proximity = colorSensor.getProximity();
    return proximity > 150;
  }
}
