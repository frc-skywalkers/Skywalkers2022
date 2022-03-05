// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class IndexBall extends CommandBase {
  /** Creates a new IndexBall. */
  private Indexer indexer;
  private boolean ballToBeIndexed = false;

  public IndexBall(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.isBallAtEntry()) {
      this.ballToBeIndexed = true;
    }
    if (indexer.isBallAtExit()) {
      this.ballToBeIndexed = false;
    }
    if (this.ballToBeIndexed) {
      indexer.setOutput(IndexerConstants.kIndexerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
