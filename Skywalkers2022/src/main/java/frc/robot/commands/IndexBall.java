// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexBall extends CommandBase {
  /** Creates a new IndexBall. */
  private Indexer indexer;
  // Intake intake;
  // Funnel funnel;
  private boolean ballToBeIndexed;

  public IndexBall(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    // this.intake = intake;
    // this.funnel = funnel;
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
      this.ballToBeIndexed = false;
    } else {
      this.ballToBeIndexed = true;
    }
    if (this.ballToBeIndexed) {
      indexer.on();
    } else {
      indexer.off();
    }
    // intake.intake();
    // funnel.setOutput(FunnelConstants.kFunnelSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // intake.setRollerOutput(0);
    // funnel.setOutput(0);
    indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
