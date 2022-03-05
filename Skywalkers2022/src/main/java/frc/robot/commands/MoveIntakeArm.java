// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntakeArm extends CommandBase {
  /** Creates a new MoveIntakeArm. */

  private Intake intake;
  private double speed;
  private double target;
  private boolean forward = true;

  public MoveIntakeArm(Intake intake, double speed, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    this.target = target;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = intake.getArmPosition();
    if (currentPosition > this.target) {
      this.speed *= -1;
      this.forward = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setArmOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = intake.getArmPosition();
    if (forward && currentPosition > target) {
      return true;
    }
    if (!forward && currentPosition < target) {
      return true;
    }
    return false;
  }
}
