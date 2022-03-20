// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterV2;

public class BringShooterToSpeed extends CommandBase {
  /** Creates a new BringShooterToSpeed. */
  ShooterV2 shooter;
  double targetRPS;

  public BringShooterToSpeed(ShooterV2 shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.targetRPS = RobotContainer.ranges[RobotContainer.rangeIndex][1];
    SmartDashboard.putNumber("Target", targetRPS);
    addRequirements(shooter);

  }

  public BringShooterToSpeed(ShooterV2 shooter, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.targetRPS = target;
    SmartDashboard.putNumber("Target", targetRPS);
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetRPS = RobotContainer.ranges[RobotContainer.rangeIndex][1];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.targetRPS = SmartDashboard.getNumber("Target", 0);
    shooter.setSpeed(this.targetRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
