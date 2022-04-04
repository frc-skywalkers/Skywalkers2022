// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance extends CommandBase {
  /** Creates a new DriveForDistance. */
  private Drivetrain drivetrain;
  private double targetDistance;
  private double kP;
  private double tolerance;
  private double targetAngle;

  public DriveForDistance(Drivetrain drivetrain, double targetDistance, double kP, double tolerance) {
    this.drivetrain = drivetrain;
    this.targetDistance = targetDistance;
    this.kP = kP;
    this.tolerance = tolerance;
    
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetDrivetrainEncoders();
    this.targetAngle = drivetrain.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dif = targetDistance - drivetrain.getAverageEncoderDistance();
    dif = MathUtil.clamp(dif * kP, -0.5, 0.5);
    double angleDif = targetAngle - drivetrain.getHeading();
    angleDif = MathUtil.clamp(angleDif * 0.02, -0.3, 0.3);
    SmartDashboard.putNumber("Angle Dif", angleDif);
    drivetrain.arcadeDrive(dif, angleDif);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getAverageEncoderDistance() - targetDistance) < tolerance;
  }
}
