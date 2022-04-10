// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignRobotShooter;
import frc.robot.commands.DiagnosticTest;
import frc.robot.commands.IMUTurn;
import frc.robot.commands.IndexBall;
import frc.robot.commands.MVPAuto;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.TwoBallAuto;
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
  public Drivetrain drive = new Drivetrain();
  private Climber climber = new Climber();
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  private Indexer indexer = new Indexer();
  private Hood hood = new Hood();
  private Shooter shooter = new Shooter();
  public Limelight limelight = new Limelight();

  // private boolean climberStarted = false;

  XboxController driverController1 = new XboxController(OIConstants.kDriverController1Port);
  XboxController driverController2 = new XboxController(OIConstants.kDriverController2Port);

  // GAMEPAD1
  // Left Y: Drivetrain Turning
  // Right X: Drivetrain Motion
  // Left Trigger: Slow Mode
  // X: Limelight Aim
  // A: Intake On
  // B: Intake Off
  // Right Bumper: Lift Arm
  // Left Bumper: Lower Arm

  // GAMEPAD2
  // Y: Disable Anti-Tip, Required for Climber Arm
  // Left Y + Y: Climber Arm
  // Right Y: Indexer
  // X: Set Shooter/Hood Targets
  // A: Shoot
  // B: Stop Shooter

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new RunCommand(
      () -> {
        // climberStarted |= (Math.abs(driverController2.getRawAxis(OIConstants.kLeftY)) > OIConstants.kDeadZone && driverController2.getRawButton(OIConstants.kY));

        if (driverController1.getRawButton(OIConstants.kX)) {
          System.out.println("x: " + limelight.getX());
          drive.arcadeDrive(0, MathUtil.clamp(limelight.getX() * 0.05, -0.6, 0.6));
        // } else if (!driverController2.getRawButton(OIConstants.kY)) {
        //   drive.arcadeDrive(MathUtil.clamp(drive.getTilt() * DriveConstants.kTiltP, -0.5, 0.5), 0);
        } else if (driverController1.getRawAxis(OIConstants.kLeftTrigger) > 0.05) {
          drive.arcadeDrive(
            -driverController1.getRawAxis(OIConstants.kLeftY) * DriveConstants.kSlowOutput,
            driverController1.getRawAxis(OIConstants.kRightX) * DriveConstants.kSlowOutput);
        } else {
          drive.arcadeDrive(
            -driverController1.getRawAxis(OIConstants.kLeftY),
            driverController1.getRawAxis(OIConstants.kRightX) * DriveConstants.kTurnOutput);
        }
      }, drive));

    climber.setDefaultCommand(new RunCommand(
      () ->
        climber.rotateArms(
          driverController2.getRawAxis(OIConstants.kLeftY),
          true),
      climber));

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

    // Change intake on to second controller up d-pad; remove on intake from first controller
    // Same thing for off intake; put it on down d-pad
    // Move deploy intake and retract to second controller

    new POVButton(driverController2, 0).whenPressed(
      new InstantCommand(() -> intake.intake(), intake)
      // .alongWith(new IndexBall(indexer))
    );

    new POVButton(driverController2, 180).whenPressed(new InstantCommand(
      () -> {
        intake.stopRollers();
        // indexer.off();
      }, intake));

    new JoystickButton(driverController2, Button.kX.value).whenPressed(new InstantCommand(() -> limelight.ledOn()).andThen(new RunCommand(
      () -> {
        // if (driverController2.getRawButton(OIConstants.kY)) {
          // shooter.setSpeed(10);
        // } else {
          hood.setPosition(MathUtil.clamp(9.18 * limelight.getDistance() / 12 - 59.9, 0, 90));
          shooter.setSpeed(MathUtil.clamp(0.503 * limelight.getDistance() / 12 + 16.8, 0, 26));
        // }
      }, shooter, hood)));

    new JoystickButton(driverController2, Button.kY.value).whenPressed(
      new InstantCommand(() -> shooter.setSpeed(20))
      .alongWith(new InstantCommand(() -> hood.setPosition(20))));

    new JoystickButton(driverController2, Button.kA.value).whenPressed(
      new RunCommand(() -> indexer.setOutput(0.9), indexer).withTimeout(2)
    );

    new JoystickButton(driverController2, Button.kB.value).whenPressed(
      new InstantCommand(() -> shooter.stopShoot(), shooter).andThen(new InstantCommand(() -> limelight.ledOff()))
    );

    new JoystickButton(driverController2, Button.kRightBumper.value).whenPressed(
      new InstantCommand(() -> intake.stopRollers())
      .andThen(new MoveArmToPosition(arm, 0, 0.125, 0.25))
      .andThen(new InstantCommand(() -> indexer.off())));

    new JoystickButton(driverController2, Button.kLeftBumper.value).whenPressed(
      new MoveArmToPosition(arm, 14, 0.075, 0.25)
      .andThen(new InstantCommand(() -> intake.intake())
      .alongWith(new InstantCommand(() -> shooter.setSpeed(0), shooter)
      .alongWith(new InstantCommand(() -> limelight.ledOff())
      .andThen(new IndexBall(indexer))
      ))));

    new JoystickButton(driverController1, Button.kX.value).whenPressed(
      new InstantCommand(() -> limelight.ledOn()).andThen( 
      new AlignRobotShooter(limelight, 0.4, 1, drive, driverController1, 0.15, 0, 0)).andThen(
      new RunCommand(() -> {setRumble(0.2);}).withTimeout(0.5)).andThen(
      new InstantCommand(() -> {stopRumble();}))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new MVPAuto(shooter, indexer, drive);
    // return new IMUTurn(drive, 175, 0.4, 1)
    // .andThen(new WaitCommand(2))
    // .andThen(new IMUTurn(drive, 5, 0.4, 1));
    return new TwoBallAuto(drive, shooter, hood, indexer, arm, intake, limelight);
    // return new DiagnosticTest(drive, shooter, hood, intake, indexer, arm, climber);
  }

  private void setRumble(double intensity) {
    driverController1.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
    driverController1.setRumble(GenericHID.RumbleType.kRightRumble, intensity);
    driverController2.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
    driverController2.setRumble(GenericHID.RumbleType.kRightRumble, intensity);
  }

  public void startRumble() {
    setRumble(0.4);
  }

  public void stopRumble() {
    setRumble(0);
  }
}
