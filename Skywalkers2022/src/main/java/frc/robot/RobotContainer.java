// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DifferentialDrive motorTester;
  public static XboxController driverJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    driverJoystick = new XboxController(Constants.JOYSTICK_PORT);


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton test = new JoystickButton(driverJoystick, Constants.JOYSTICK_PORT);
  } 

  private void testSparkMAX(int portNumber, int maxSpeed) {
    CANSparkMax m_motor = new CANSparkMax(portNumber, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(false);
    while(true) {
      m_motor.set(driverJoystick.getRawAxis(Constants.XBOX_LEFT_X_AXIS) * maxSpeed);
    }
  }

  private void testTalonFX(int portNumber, int maxSpeed) {
    WPI_TalonFX m_motor = new WPI_TalonFX(portNumber);
    m_motor.configFactoryDefault();
    m_motor.setInverted(false);
    while(true) {
      m_motor.set(driverJoystick.getRawAxis(Constants.XBOX_LEFT_X_AXIS) * maxSpeed);
    }
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
