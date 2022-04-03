// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DiagnosticTest extends SequentialCommandGroup {
  /** Creates a new DiagnosticTest. */

  public DiagnosticTest(Drivetrain drivetrain, Shooter shooter, Hood hood, Intake intake, Indexer indexer, Arm arm) {
    addCommands(new SequentialCommandGroup(
      new RunCommand(() -> drivetrain.arcadeDrive(0.5, 0), drivetrain).withTimeout(1),
      new InstantCommand(() -> drivetrain.stop(), drivetrain),
      new MoveHood(hood, 100),
      new MoveHood(hood, 0),
      new BringShooterToSpeed(shooter, 25),
      new WaitCommand(2),
      new InstantCommand(() -> shooter.stopShoot(), intake),
      new MoveArmToPosition(arm, 10, 0.125, 1),
      new WaitCommand(1),
      new RunCommand(() -> intake.intake(), intake).withTimeout(1.5),
      new InstantCommand(() -> intake.stopRollers(), intake),
      new WaitCommand(1),
      new MoveArmToPosition(arm, 0, 0.075, 1),
      new WaitCommand(1),
      new RunCommand(() -> indexer.on(), indexer).withTimeout(1.5),
      new InstantCommand(() -> indexer.off(), indexer)
    ));
  }
}
