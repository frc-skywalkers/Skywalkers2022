// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // Create motor objects 

  public Intake() {
    // initialize motors + encoders
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reset() {
    setOutput(0);
    // reset intake arm position encoders
  }

  public double getArmPosition() { 
    return -1;
  }


  public void setOutput(double speed) {
    // Set the output of the rollers to @param speed
  }

  public double getOutput() {
    // return roller speed
    return -1;
  }

  public void toggleRollers(boolean enabled) {
    if (enabled && getOutput() == 0) {

    }
  }

  public boolean isDeployed() {
    if (getArmPosition() >= IntakeConstants.kArmThreshold) {
      return true;
    } else {
      return false;
    }
  }

  public void extendIntake(boolean extend) {
    if (extend && !isDeployed()) {
      // push intake out
    } else {
      // pull intake in
    }
  }

  public void deployIntake() {
    extendIntake(true);
    toggleRollers(true);
  }

  public void retractIntake() {
    extendIntake(false);
    toggleRollers(true);
  }


}
