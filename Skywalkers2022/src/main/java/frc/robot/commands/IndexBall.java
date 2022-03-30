// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexBall extends CommandBase {
  /** Creates a new IndexBall. */
  private Indexer indexer;
  // Intake intake;
  private boolean ballToBeIndexed;

  public IndexBall(Indexer indexer) {
    this.indexer = indexer;
    // this.intake = intake;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballToBeIndexed = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.isBallAtExit()) {
      ballToBeIndexed = false;
    } else {
      ballToBeIndexed = true;
    }

    if (ballToBeIndexed) {
      indexer.on();
    } else {
      indexer.off();
    }
    // intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // intake.stopRollers();
    indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
