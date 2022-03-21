// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class MoveHood extends CommandBase {
  /** Creates a new MoveHood. */
  private Hood hood;
  private double targetPos;

  public MoveHood(Hood hood, double target) {
    this.hood = hood;
    targetPos = target;
    SmartDashboard.putNumber("Target Hood", targetPos);
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // targetPos = SmartDashboard.getNumber("Target Hood", 0);
    hood.setPosition(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.atPosition(1);
  }
}
