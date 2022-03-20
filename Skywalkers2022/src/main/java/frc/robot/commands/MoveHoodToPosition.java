// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Hood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveHoodToPosition extends PIDCommand {
  /** Creates a new MoveHoodToPosition. */

  public MoveHoodToPosition(Hood hood, double targetPosition) {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0, 0),
        // This should return the measurement
        hood::getPosition,
        // This should return the setpoint (can also be a constant)
        SmartDashboard.getNumber("Hood Target", 50),
        // This uses the output
        output -> hood.setOutput(output),
        
        hood);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(hood);
    getController().setTolerance(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
