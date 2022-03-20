// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BringShooterToSpeed;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.IndexBall;
import frc.robot.commands.MVPAuto;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveHood;
import frc.robot.commands.MoveHoodToPosition;
import frc.robot.commands.Shoot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterV2;
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
  // private Shooter shooter = new Shooter();
  private ShooterV2 shooterV2 = new ShooterV2();

  public static double[][] ranges = {{0,20}, {50, 20}, {100, 25}};
  public static int rangeIndex = 0;

  XboxController driverController1 = new XboxController(OIConstants.kDriverController1Port);
  XboxController driverController2 = new XboxController(OIConstants.kDriverController2Port);

  // Test web hook
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand( 
        new RunCommand(
            () ->
              drive.arcadeDrive(
                  -driverController1.getRawAxis(OIConstants.kLeftY),
                  driverController1.getRawAxis(OIConstants.kRightX)),
            drive));

    // drive.setDefaultCommand( 
    //     new RunCommand(
    //         () ->
    //           drive.setLeft(-driverController1.getRawAxis(OIConstants.kLeftY)), 
    //         drive));
    
    // climber.setDefaultCommand(
    //   new RunCommand(
    //     () ->
    //       climber.rotateArms(driverController2.getRawAxis(OIConstants.kRightY)),
    //     climber));

    hood.setDefaultCommand(
      new RunCommand(
        () -> 
          hood.setOutput(-driverController2.getRawAxis(OIConstants.kLeftY) * 0.2), 
      hood));

    // shooter.setDefaultCommand(
    //   new RunCommand(
    //     () -> 
    //       shooter.setVoltage(-driverController2.getRawAxis(OIConstants.kRightY) * 12), 
    //     shooter));

    indexer.setDefaultCommand(
      new RunCommand(
        () -> 
          indexer.setOutput(-driverController2.getRawAxis(OIConstants.kRightY) * 0.7), 
        indexer));

    // indexer.setDefaultCommand(new IndexBall(indexer));

    // indexer.setDefaultCommand(new IndexBall(indexer));

    // funnel.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       if (intake.isDeployed()) {
    //         funnel.setOutput(FunnelConstants.kFunnelSpeed);
    //       }
    //     }, funnel)
    // );
    
    // climber.setDefaultCommand(
    //   new RunCommand(
    //       () ->
    //         climber.move(driverController.getRawButton(OIConstants.kA), driverController.getRawButton(OIConstants.kB),
    //         driverController.getRawButton(OIConstants.kX), driverController.getRawButton(OIConstants.kY),
    //         driverController.getRawButton(OIConstants.kLeftBumper), driverController.getRawButton(OIConstants.kRightBumper),
    //         driverController.getRawAxis(OIConstants.kRightY), driverController.getPOV()), climber));

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

    // new JoystickButton(driverController2, driverController2.getPOV())

    new JoystickButton(driverController1, OIConstants.kStopRollerButton.value).whenPressed(new InstantCommand(
      () -> {
        intake.stopRollers();
        // funnel.setOutput(0);
        indexer.setOutput(0);
      },
      indexer, intake, funnel));

    new POVButton(driverController2, 0).whenPressed(() -> {
      rangeIndex = (rangeIndex + 1)%3;
      System.out.println("D-PAD Top");
      System.out.println("Range Index " + rangeIndex);
      switch (rangeIndex) {
        case 0: SmartDashboard.putString("Shooter Range", "Low");
                break;
        case 1: SmartDashboard.putString("Shooter Range", "Mid");
                break;
        case 2: SmartDashboard.putString("Shooter Range", "High");
                break;
        default: SmartDashboard.putString("Shooter Range", "Unknown");
                break;
        
      }
    });
    new POVButton(driverController2, 180).whenPressed(() -> {
      rangeIndex = (rangeIndex + 2)%3;
      System.out.println("D-PAD Bottom");
      System.out.println("Range Index " + rangeIndex);
      switch (rangeIndex) {
        case 0: SmartDashboard.putString("Shooter Range", "Low");
                break;
        case 1: SmartDashboard.putString("Shooter Range", "Mid");
                break;
        case 2: SmartDashboard.putString("Shooter Range", "High");
                break;
        default: SmartDashboard.putString("Shooter Range", "Unknown");
                break;
        
      }
    });

    // new JoystickButton(driverController2, 8).whenPressed(
    //   new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       new MoveHood(hood, 50),
    //       new BringShooterToSpeed(shooterV2, 20)
    //     ),
    //     new Shoot(indexer),
    //     new InstantCommand(() -> shooterV2.setVoltage(0.0))
    // ));

    // new JoystickButton(driverController2, Button.kX.value).whenPressed(new BringShooterToSpeed(shooterV2, 20)); 
    // new JoystickButton(driverController2, Button.kY.value).whenPressed(new InstantCommand(shooterV2::stopShoot, shooterV2));
    
    new JoystickButton(driverController2, Button.kX.value).whenPressed(
      new MoveHood(hood).alongWith(
      new BringShooterToSpeed(shooterV2)
      // .raceWith(
      // new SequentialCommandGroup(
        // new WaitUntilCommand(() -> shooterV2.atSpeed(1)),
        // new WaitUntilCommand(() -> hood.atPosition(1)),
        // new RunCommand(() -> indexer.setOutput(IndexerConstants.kIndexerSpeed), indexer).withTimeout(2),
        // new InstantCommand(() -> indexer.setOutput(0))))
      )
    ); 

    new JoystickButton(driverController2, Button.kA.value).whenPressed(
        new RunCommand(
          () -> indexer.setOutput(0.9),
        indexer).withTimeout(2)
    );

    new JoystickButton(driverController2, Button.kB.value).whenPressed(
      new InstantCommand(() -> shooterV2.setVoltage(0), shooterV2)
    );
    
    
    // new JoystickButton(driverController2, Button.kX.value).whenPressed(new MoveHood(hood, 50)); 
    // new JoystickButton(driverController2, Button.kY.value).whenPressed(new InstantCommand(() -> hood.setOutput(0), hood));
    

    // new JoystickButton(driverController2, OIConstants.kLiftArmButton.value).whileHeld(() -> arm.arm(ArmConstants.kLiftArmSpeed));
    // new JoystickButton(driverController2, OIConstants.kLowerArmButton.value).whileHeld(() -> arm.arm(ArmConstants.kLowerArmSpeed));
    // new JoystickButton(driverController2, OIConstants.kLiftArmButton.value).whenReleased(() -> arm.stop());
    // new JoystickButton(driverController2, OIConstants.kLowerArmButton.value).whenReleased(() -> arm.stop());
    new JoystickButton(driverController1, OIConstants.kLiftArmButton.value).whenPressed(new MoveArmToPosition(arm, 0, 0.1, 0.25));
    new JoystickButton(driverController1, OIConstants.kLowerArmButton.value).whenPressed(new MoveArmToPosition(arm, 14, 0.05, 0.25));

    // new JoystickButton(driverController2, OIConstants.kUnlatchFirstRungButton.value).whenPressed(() -> climber.unlatchFirst());
    // new JoystickButton(driverController2, OIConstants.kUnlatchSecondRungButton.value).whenPressed(() -> climber.unlatchSecond());
    // new JoystickButton(driverController2, OIConstants.kUnlatchFirstRungButton.value).whenReleased(() -> climber.latchFirst());
    // new JoystickButton(driverController2, OIConstants.kUnlatchSecondRungButton.value).whenReleased(() -> climber.latchSecond());
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new DriveForDistance(drive, Units.feetToMeters(-5), 1, 0.05);
    return new MVPAuto(shooterV2, indexer, drive);
  }
}
