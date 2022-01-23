// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
  /** Creates a new Shooter. */

  // create motor objects


  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    // instantiate motor objects + encoders
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void setShooterRPS(double rps) {
    this.setSetpoint(rps);
  }

  public double getSpeed() {
    // return encoder rps
    return -1;
  }

  public void reset() {
    // reset encoders
  }

  public void getHoodPosition() {

  }

  public void moveHood(double speed) {
    // set hood motor to speed
  }

}
