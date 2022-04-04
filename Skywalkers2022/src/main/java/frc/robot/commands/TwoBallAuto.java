// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */

  public TwoBallAuto(Shooter shooter, Hood hood, Arm arm, Indexer indexer, Intake intake, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetIMU()),
      new BringShooterToSpeed(shooter, 20.2).alongWith(new MoveHood(hood, 2.1)),
      new RunCommand(() -> indexer.setOutput(0.9), indexer).withTimeout(2),
      new InstantCommand(() -> indexer.off(), indexer),
      new InstantCommand(() -> shooter.stopShoot(), shooter),
      // new IndexBall(indexer).alongWith(new DriveForDistance(drivetrain, Units.feetToMeters(5), 1, 0.1)),
      new IMUTurn(drivetrain, 180, 0.2, 2),
      new MoveArmToPosition(arm, 14, 0.075, 0.25),
      new InstantCommand(() -> intake.intake()),
      new InstantCommand(() -> indexer.setOutput(0.9)),
      new DriveForDistance(drivetrain, Units.feetToMeters(4), 1, 0.3),
      new InstantCommand(() -> intake.stopRollers()),
      new InstantCommand(() -> indexer.off()),
      new MoveArmToPosition(arm, 0, 0.125, 0.25),
      new IMUTurn(drivetrain, 0, 0.2, 2),
      new BringShooterToSpeed(shooter, 21.9).alongWith(new MoveHood(hood, 33.4)),
      new RunCommand(() -> indexer.setOutput(0.9), indexer).withTimeout(2),
      new InstantCommand(() -> indexer.off(), indexer),
      new InstantCommand(() -> shooter.stopShoot(), shooter),
      new DriveForDistance(drivetrain, Units.feetToMeters(-5), 1, 0.3),
      // new BringShooterToSpeed(shooter, 20),
      // new RunCommand(() -> indexer.on(), indexer).withTimeout(2),
      // new InstantCommand(() -> indexer.off()),

      new InstantCommand(() -> {System.out.println("Done");})
    ));
  }
}
