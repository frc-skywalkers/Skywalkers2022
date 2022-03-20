// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class MoveHood extends CommandBase {
  /** Creates a new MoveHood. */
  double targetPos;
  Hood hood;
  public MoveHood(Hood curHood) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.hood = curHood;
    SmartDashboard.putNumber("Target Hood", targetPos);
    addRequirements(hood);
  }

  // public MoveHood(Hood curHood, double tPos) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   this.targetPos = tPos;
  //   this.hood = curHood;
  //   SmartDashboard.putNumber("Target Hood", targetPos);
  //   addRequirements(hood);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetPos = RobotContainer.ranges[RobotContainer.rangeIndex][0];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.targetPos = SmartDashboard.getNumber("Target Hood", 0);
    this.hood.setPosition(this.targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.atPosition(targetPos, 1);
  }
}
