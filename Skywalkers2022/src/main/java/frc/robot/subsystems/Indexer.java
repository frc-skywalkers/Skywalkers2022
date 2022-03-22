// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  private CANSparkMax indexerMotor = new CANSparkMax(IndexerConstants.kMotorPort, MotorType.kBrushless);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private RelativeEncoder indexerEncoder;
  
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch colorMatcher = new ColorMatch();
  // private final Color kBlue = new Color(0, 0, 0);
  // private final Color kRed = new Color(0, 0, 0);
  // TODO: implement beam breaker sensors

  public enum BallColor {
    BLUE, RED, UKNOWN;
  }

  public Indexer() {

    indexerMotor.restoreFactoryDefaults();
    indexerMotor.setInverted(IndexerConstants.kInvert);
    indexerEncoder = indexerMotor.getEncoder();
   
    // colorMatcher.addColorMatch(kBlue);
    // colorMatcher.addColorMatch(kRed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    SmartDashboard.putNumber("Indexer Velocity", getVelocity());
  }

  public void setOutput(double speed) {
    indexerMotor.set(speed);
    SmartDashboard.putNumber("Indxer Current", indexerMotor.getOutputCurrent());
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

  // public BallColor getColor() {
  //   Color detectedColor = colorSensor.getColor();
  //   ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
  //   if (match.color == kBlue) { 
  //     return BallColor.BLUE; 
  //   } else if (match.color == kRed) {
  //     return BallColor.RED;
  //   } else {
  //     return BallColor.UKNOWN;
  //   }

  // }

  public boolean isBallAtEntry() {
    // return true if bottom beam breaker is broken
    return false;
  }
  
  public boolean isBallAtExit() {
    double proximity = colorSensor.getProximity();
    return proximity > 150;
  }
  
  public double getVelocity() {
    return indexerEncoder.getVelocity();
  }
}
