// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterV2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MVPAuto extends SequentialCommandGroup {
  /** Creates a new MVPAuto. */
  public MVPAuto(ShooterV2 shooter, Indexer indexer, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(
      new BringShooterToSpeed(shooter, 20).raceWith(
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> shooter.atSpeed(19, 1)),
        new RunCommand(() -> indexer.setOutput(IndexerConstants.kIndexerSpeed), indexer).withTimeout(2),
        new InstantCommand(() -> indexer.setOutput(0)),
        new InstantCommand(() -> { System.out.println("Done");})
      )
    )), new DriveForDistance(drivetrain, Units.feetToMeters(-5), 1, 0.1));
  }
}
