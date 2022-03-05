// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleIntake extends SequentialCommandGroup {
  /** Creates a new ToggleIntake. */

  public ToggleIntake(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (intake.isDeployed()) {
      addCommands(
        new RunCommand(() -> intake.setRollerOutput(0), intake),
        new MoveIntakeArm(intake, 0.1, IntakeConstants.kMinArmThreshold)
      );
    } else {
      addCommands(
        new MoveIntakeArm(intake, 0.1, IntakeConstants.kMaxArmThreshold),
        new RunCommand(() -> intake.setRollerOutput(0.5), intake)
      );
    }
    
  }
}
