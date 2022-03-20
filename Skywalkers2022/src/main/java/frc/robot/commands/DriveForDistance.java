// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance extends CommandBase {
  /** Creates a new DriveForDistance. */
  Drivetrain drivetrain;
  double targetDistance;
  double kP;
  double tolerance;
  public DriveForDistance(Drivetrain drivetrain, double targetDistance, double kP, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.targetDistance = targetDistance;
    this.kP = kP;
    this.tolerance = tolerance;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.drivetrain.resetDrivetrainEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curDist = this.drivetrain.getAverageEncoderDistance();
    double dif = targetDistance - curDist;
    dif = MathUtil.clamp(dif, -0.5, 0.5);
    this.drivetrain.arcadeDrive(dif * kP, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.drivetrain.getAverageEncoderDistance() - this.targetDistance) < tolerance;
  }
}
