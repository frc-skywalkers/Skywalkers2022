// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignRobotShooter extends CommandBase {
  /** Creates a new AlignRobotShooter. */
  Limelight limelight;
  Drivetrain drivetrain;
  double kP;
  double kI;
  double kD;
  double tolerance;
  double error;
  double lastTimeStamp = 0;
  double lastError = 0;
  double errorSum = 0;
  double iLimit = 0;
  XboxController tJoystick;

  public AlignRobotShooter(Limelight limelight, double kP, double tolerance, Drivetrain drivetrain, XboxController tJoystick, double kD, double kI, double iLimit) {
    this.limelight = limelight;
    this.kP = kP;
    this.tolerance = tolerance;
    this.drivetrain = drivetrain;
    this.tJoystick = tJoystick;
    this.kI = kI;
    this.kD = kD;
    this.iLimit = iLimit;

    addRequirements(drivetrain);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
// ang = limelight.getX();
    
    // double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    // double speed = MathUtil.clamp(ang * kP, -0.3, 0.3);

    // drivetrain.arcadeDrive(0, speed);
    error = limelight.getX();
    
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    double errorRate = (error - lastError) / dt;

    if(Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double outputRate = error * kP + errorRate * kD + errorSum * kI;

    outputRate = MathUtil.clamp(outputRate, -0.4, 0.4);
    drivetrain.arcadeDrive(0, outputRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= tolerance || Math.abs(tJoystick.getRawAxis(OIConstants.kRightX)) > OIConstants.kDeadZone;
  }
}
