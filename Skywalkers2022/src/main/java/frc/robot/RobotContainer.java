// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DiagnosticTest;
import frc.robot.commands.IndexBall;
import frc.robot.commands.MVPAuto;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
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
  private Indexer indexer = new Indexer();
  private Hood hood = new Hood();
  private Shooter shooter = new Shooter();
  private Limelight limelight = new Limelight();

  private SlewRateLimiter filter = new SlewRateLimiter(0.5);

  // private static double[][] ranges = {{8, 22}, {45, 20}, {90, 25}}; // {hood, shooter}
  // private static int numPoints = 3;
  // private static int rangeIndex = 0;
  // private static String[] rangeLabels = {"Low", "Mid", "High"};

  // private static double[][] ranges = {{0, 20}, {0, 22}, {8, 22}, {0, 10}, {45, 20}, {45, 22}}; // {hood, shooter}
  // private static int numPoints = 6;
  // private static int rangeIndex = 3;
  // private static String[] rangeLabels = {"Low 1", "Low 2", "Low 3", "Miss", "Mid 1", "Mid 2"};

  private static double[][] ranges = {{8, 22}, {0, 10}}; // {hood, shooter}
  private static int numPoints = 2;
  private static int rangeIndex = 0;
  private static String[] rangeLabels = {"Low 3", "Miss"};

  private boolean climberStarted = false;

  XboxController driverController1 = new XboxController(OIConstants.kDriverController1Port);
  XboxController driverController2 = new XboxController(OIConstants.kDriverController2Port);

  // GAMEPAD1
  // Left Y: Drivetrain Turning
  // Right X: Drivetrain Motion
  // X: Limelight Aim (Commented)
  // A: Intake On
  // B: Intake Off
  // Right Bumper: Lift Arm
  // Left Bumper: Lower Arm

  // GAMEPAD2
  // Left Y + Y: Climber Arm
  // Right Y: Indexer
  // DPAD Up: Increment Range
  // DPAD Down: Decrement Range
  // X: Set Shooter/Hood Targets
  // A: Shoot
  // B: Stop Shooter
  // Left Bumper: Unlatch Climber First Rung
  // Right Bumper: Unlatch Climber Second Rung

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new RunCommand(
      () -> {
        climberStarted |= (Math.abs(driverController2.getRawAxis(OIConstants.kLeftY)) > OIConstants.kDeadZone && driverController2.getRawButton(OIConstants.kY));

        if (driverController1.getRawButton(OIConstants.kX)) {
          System.out.println("x: " + limelight.getX());
          drive.arcadeDrive(0, MathUtil.clamp(limelight.getX() * 0.05, -0.6, 0.6));
        } else if (drive.isTipping() && !climberStarted) {
          drive.arcadeDrive(drive.getTilt() * -DriveConstants.kTiltP, 0);
        } else if (driverController1.getRawAxis(OIConstants.kLeftTrigger) > 0.05) {
          drive.arcadeDrive(
            -filter.calculate(driverController1.getRawAxis(OIConstants.kLeftY)) * DriveConstants.kSlowOutput,
            driverController1.getRawAxis(OIConstants.kRightX) * DriveConstants.kSlowOutput * DriveConstants.kTurnOutput);
        } else {
          drive.arcadeDrive(
            -filter.calculate(driverController1.getRawAxis(OIConstants.kLeftY)),
            driverController1.getRawAxis(OIConstants.kRightX) * DriveConstants.kTurnOutput);
        }
      }, drive));

    climber.setDefaultCommand(new RunCommand(
      () ->
        climber.rotateArms(
          driverController2.getRawAxis(OIConstants.kLeftY),
          driverController2.getRawButton(OIConstants.kY)),
      climber));

    hood.setDefaultCommand(new RunCommand(
      () -> 
        hood.setOutput(-driverController2.getRawAxis(OIConstants.kLeftY) * 0.2, !driverController2.getRawButton(OIConstants.kY)),
      hood));

    indexer.setDefaultCommand(new RunCommand(
      () -> 
        indexer.setOutput(-driverController2.getRawAxis(OIConstants.kRightY) * 0.7), 
      indexer));

    limelight.setDefaultCommand(new RunCommand(
      () ->
        limelight.updateValues(),
      limelight));

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
    // new JoystickButton(driverController1, Button.kA.value).whenPressed(new IndexBall(indexer));

    new JoystickButton(driverController1, Button.kA.value).whenPressed(intake::intake, intake);

    new JoystickButton(driverController1, Button.kB.value).whenPressed(new InstantCommand(
      () -> {
        intake.stopRollers();
        indexer.off();
      }, indexer, intake));

    new POVButton(driverController2, 0).whenPressed(() -> {
      rangeIndex = (rangeIndex + 1) % numPoints;
      // hood.setPosition(ranges[rangeIndex][0]);
      shooter.setSpeed(ranges[rangeIndex][1]);

      System.out.println("D-PAD Top, Range Index " + rangeIndex);
      SmartDashboard.putString("Shooter Range", rangeLabels[rangeIndex]);
    });
    
    new POVButton(driverController2, 180).whenPressed(() -> {
      rangeIndex = (rangeIndex - 1 + numPoints) % numPoints;
      // hood.setPosition(ranges[rangeIndex][0]);
      shooter.setSpeed(ranges[rangeIndex][1]);
 
      System.out.println("D-PAD Bottom, Range Index " + rangeIndex);
      SmartDashboard.putString("Shooter Range", rangeLabels[rangeIndex]);
    });

    new JoystickButton(driverController2, Button.kX.value).whenPressed(() -> {
      // hood.setPosition(ranges[rangeIndex][0]);
      shooter.setSpeed(ranges[rangeIndex][1]);
    });

    new JoystickButton(driverController2, Button.kA.value).whenPressed(
        new RunCommand(() -> indexer.setOutput(0.9), indexer).withTimeout(2)
    );

    new JoystickButton(driverController2, Button.kB.value).whenPressed(
      new InstantCommand(() -> shooter.stopShoot(), shooter)
    );

    // make sure comment out either lines 189, 191

    new JoystickButton(driverController1, Button.kRightBumper.value).whenPressed(new DiagnosticTest(drive, shooter, hood, intake, indexer, arm));
    
    // new JoystickButton(driverController1, Button.kRightBumper.value).whenPressed(new MoveArmToPosition(arm, 0, 0.125, 0.25));
    new JoystickButton(driverController1, Button.kLeftBumper.value).whenPressed(new MoveArmToPosition(arm, 14, 0.075, 0.25));

    new JoystickButton(driverController2, Button.kLeftBumper.value).whenPressed(() -> climber.unlatchFirst());
    new JoystickButton(driverController2, Button.kLeftBumper.value).whenReleased(() -> climber.latchFirst());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new MVPAuto(shooter, indexer, drive);
  }

  private void setRumble(double intensity) {
    driverController1.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
    driverController1.setRumble(GenericHID.RumbleType.kRightRumble, intensity);
    driverController2.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
    driverController2.setRumble(GenericHID.RumbleType.kRightRumble, intensity);
  }

  public void startRumble() {
    setRumble(0.5);
  }

  public void stopRumble() {
    setRumble(0);
  }
}
