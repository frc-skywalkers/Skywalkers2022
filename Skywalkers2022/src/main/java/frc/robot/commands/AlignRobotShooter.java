// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignRobotShooter extends CommandBase {
  /** Creates a new AlignRobotShooter. */


  Limelight limelight;
  Drivetrain drivetrain;
  double kP;
  double tolerance;
  double ang;


  public AlignRobotShooter(Limelight limelight, double kP, double tolerance, Drivetrain drivetrain) {
    this.limelight = limelight;
    this.kP = kP;
    this.tolerance = tolerance;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    addRequirements(limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ang = limelight.getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ang = limelight.getX();
    double speed = ang * kP;
    speed = MathUtil.clamp(speed, -0.3, 0.3);
    drivetrain.arcadeDrive(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(ang) <= tolerance;
  }
}
