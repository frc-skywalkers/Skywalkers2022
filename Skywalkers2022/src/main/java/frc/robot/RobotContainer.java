// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain drive = new Drivetrain(); 
  private Climber climber = new Climber();
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand( 
        new RunCommand(
            () ->
              drive.arcadeDrive(
                  -driverController.getRawAxis(OIConstants.kLeftY),
                  driverController.getRawAxis(OIConstants.kRightX)),
            drive));
    
    // climber.setDefaultCommand(
    //   new RunCommand(
    //       () ->
    //         climber.move(driverController.getRawAxis(OIConstants.kLeftY), driverController.getRawAxis(OIConstants.kRightY)),
    //       climber)); //change controller constants

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, OIConstants.kIntakeButton.value).whenPressed(intake::intake);
    new JoystickButton(driverController, OIConstants.kStopRollerButton.value).whenPressed(intake::stop);
    new JoystickButton(driverController, OIConstants.kLiftArmButton.value).whileHeld(() -> arm.arm(ArmConstants.kLiftArmSpeed));
    new JoystickButton(driverController, OIConstants.kLowerArmButton.value).whileHeld(() -> arm.arm(ArmConstants.kLowerArmSpeed));
    new JoystickButton(driverController, OIConstants.kLiftArmButton.value).whenReleased(() ->arm.stop());
    new JoystickButton(driverController, OIConstants.kLowerArmButton.value).whenReleased(() -> arm.stop());

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
