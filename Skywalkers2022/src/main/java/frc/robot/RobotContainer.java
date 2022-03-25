// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BringShooterToSpeed;
import frc.robot.commands.IndexBall;
import frc.robot.commands.MVPAuto;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveHood;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Funnel;
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
  private Funnel funnel = new Funnel();
  private Hood hood = new Hood();
  private Shooter shooter = new Shooter();
  private Limelight limelight = new Limelight();

  // private static double[][] ranges = {{8, 22}, {45, 20}, {90, 25}}; // {hood, shooter}
  // private static int numPoints = 3;
  // private static int rangeIndex = 0;
  // private static String[] rangeLabels = {"Low", "Mid", "High"};

  private static double[][] ranges = {{0, 20}, {0, 22}, {8, 22}, {0, 10}, {45, 20}, {45, 22}}; // {hood, shooter}
  private static int numPoints = 6;
  private static int rangeIndex = 3;
  private static String[] rangeLabels = {"Low 1", "Low 2", "Low 3", "Miss", "Mid 1", "Mid 2"};

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
    drive.setDefaultCommand( 
      new RunCommand(
        () ->
          drive.arcadeDrive(
            -driverController1.getRawAxis(OIConstants.kLeftY),
            driverController1.getRawAxis(OIConstants.kRightX) * 0.7),
        drive));

    climber.setDefaultCommand(
      new RunCommand(
        () ->
          climber.rotateArms(
            driverController2.getRawAxis(OIConstants.kLeftY),
            driverController2.getRawButton(OIConstants.kY)),
        climber));

    // hood.setDefaultCommand(
    //   new RunCommand(
    //     () -> 
    //       hood.setOutput(-driverController2.getRawAxis(OIConstants.kLeftY) * 0.2), // change control (interferes with climber arms)
    //     hood));

    indexer.setDefaultCommand(
      new RunCommand(
        () -> 
          indexer.setOutput(-driverController2.getRawAxis(OIConstants.kRightY) * 0.7), 
        indexer));

    // indexer.setDefaultCommand(new IndexBall(indexer));

    // funnel.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       if (intake.isDeployed()) {
    //         funnel.on();
    //       }
    //     }, funnel)
    // );

    limelight.setDefaultCommand(
      new RunCommand(
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
    new JoystickButton(driverController1, OIConstants.kIntakeButton.value).whenPressed(new IndexBall(indexer));
    new JoystickButton(driverController1, OIConstants.kIntakeButton.value).whenPressed(intake::intake, intake);

    new JoystickButton(driverController1, OIConstants.kStopRollerButton.value).whenPressed(new InstantCommand(
      () -> {
        intake.stopRollers();
        // funnel.off();
        indexer.off();
      },
      indexer, intake, funnel));

    new JoystickButton(driverController1, Button.kX.value).whenHeld(new InstantCommand(
      () -> {
        System.out.println("x: " + limelight.getX());
        drive.arcadeDrive(0, MathUtil.clamp(limelight.getX() * 0.05, -0.6, 0.6));
      },
      drive));

    new POVButton(driverController2, 0).whenPressed(() -> {
      rangeIndex = (rangeIndex + 1) % numPoints;
      hood.setPosition(ranges[rangeIndex][0]);
      shooter.setSpeed(ranges[rangeIndex][1]);

      System.out.println("D-PAD Top, Range Index " + rangeIndex);
      SmartDashboard.putString("Shooter Range", rangeLabels[rangeIndex]);
    });
    
    new POVButton(driverController2, 180).whenPressed(() -> {
      rangeIndex = (rangeIndex - 1 + numPoints) % numPoints;
      hood.setPosition(ranges[rangeIndex][0]);
      shooter.setSpeed(ranges[rangeIndex][1]);
 
      System.out.println("D-PAD Bottom, Range Index " + rangeIndex);
      SmartDashboard.putString("Shooter Range", rangeLabels[rangeIndex]);
    });

    // new JoystickButton(driverController2, Button.kX.value).whenPressed(
    //   new MoveHood(hood, ranges[rangeIndex][0]).alongWith(
    //   new BringShooterToSpeed(shooter, ranges[rangeIndex][1]))
    //   // .andThen(
    //   // new SequentialCommandGroup(
    //     // new RunCommand(() -> indexer.on(), indexer).withTimeout(2),
    //     // new InstantCommand(() -> indexer.off())))
    // );

    new JoystickButton(driverController2, Button.kX.value).whenPressed(() -> {
      hood.setPosition(ranges[rangeIndex][0]);
      shooter.setSpeed(ranges[rangeIndex][1]);
    });

    new JoystickButton(driverController2, Button.kA.value).whenPressed(
        new RunCommand(() -> indexer.setOutput(0.9), indexer).withTimeout(2)
    );

    new JoystickButton(driverController2, Button.kB.value).whenPressed(
      new InstantCommand(() -> shooter.stopShoot(), shooter)
    );
    
    new JoystickButton(driverController1, OIConstants.kLiftArmButton.value).whenPressed(new MoveArmToPosition(arm, 0, 0.1, 0.25));
    new JoystickButton(driverController1, OIConstants.kLowerArmButton.value).whenPressed(new MoveArmToPosition(arm, 14, 0.05, 0.25));

    new JoystickButton(driverController2, Button.kLeftBumper.value).whenPressed(() -> climber.unlatchFirst());
    new JoystickButton(driverController2, Button.kLeftBumper.value).whenReleased(() -> climber.latchFirst());
    // new JoystickButton(driverController2, Button.kRightBumper.value).whenPressed(() -> climber.unlatchSecond());
    // new JoystickButton(driverController2, Button.kRightBumper.value).whenReleased(() -> climber.latchSecond());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new MVPAuto(shooter, indexer, drive);
  }
}
