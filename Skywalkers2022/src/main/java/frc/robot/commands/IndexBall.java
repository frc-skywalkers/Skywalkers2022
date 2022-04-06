// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexBall extends CommandBase {
  /** Creates a new IndexBall. */
  private Indexer indexer;
  private boolean ballToBeIndexed = true;

  public IndexBall(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.on();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.isBallAtExit();
  }
}
