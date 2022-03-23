// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class BringShooterToSpeed extends CommandBase {
  /** Creates a new BringShooterToSpeed. */
  private Shooter shooter;
  private double targetRPS;

  public BringShooterToSpeed(Shooter shooter, double target) {
    this.shooter = shooter;
    targetRPS = target;
    SmartDashboard.putNumber("Target", targetRPS);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // targetRPS = SmartDashboard.getNumber("Target", 0);
    shooter.setSpeed(targetRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.atSpeed(1);
  }
}
