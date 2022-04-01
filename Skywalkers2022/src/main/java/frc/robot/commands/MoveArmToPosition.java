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
  private Arm arm;
  private double targetArmPos;
  private double kP;
  private double tolerance;

  public MoveArmToPosition(Arm arm, double targetArmPos, double kP, double tolerance) {
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
    double dif = targetArmPos - arm.getPosition();
    double speed = MathUtil.clamp(kP * dif, -0.25, 0.25);
    SmartDashboard.putNumber("Arm Target", targetArmPos);
    arm.arm(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getPosition() - targetArmPos) < tolerance;
  }
}
