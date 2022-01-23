// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  // create motor objects
  // create distance sensor
  // create color sensor
  public Indexer() {
    // intialize motor objects + encoders
    // perform distance sensor setup
    // perform color sensor setup
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reset() {
    // reset encoders
    setOutput(0);
  }

  public double getSensorDistance() {
    // return sensor distance
    return -1;
  }

  public int getBallCount() {
    // use getSensorDistance() to calculate number of balls in indexer
    return -1;
  }

  public Color getColor() {
    // returns color of top ball or no color if no ball at top
    return new Color(-0, -0, -0);
  }

  public void setOutput(double speed) {
    // set speed of indexer motor to speed
  }

  public double getPosition() {
    // return position of encoder
    return -1;
  }

  


}
