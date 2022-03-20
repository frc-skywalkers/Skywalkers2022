// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends CommandBase {
  /** Creates a new MoveArmToPosition. */
  Arm arm;
  double targetArmPos;
  double kP;
  double tolerance;

  public MoveArmToPosition(Arm arm, double targetArmPos, double kP, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.targetArmPos = targetArmPos;
    this.kP = kP;
    this.tolerance = tolerance;
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curPos = this.arm.getPosition();
    double dif = this.targetArmPos - curPos;
    double speed = this.kP * dif;
    speed = MathUtil.clamp(speed, -0.1, 0.1);
    SmartDashboard.putNumber("Arm Power", speed);
    this.arm.arm(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.arm.getPosition() - this.targetArmPos) < this.tolerance;
  }
}
