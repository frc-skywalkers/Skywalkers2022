// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DiagnosticTest extends CommandBase {
  /** Creates a new DiagnosticTest. */

  private Drivetrain drivetrain;
  private Shooter shooter;
  private Hood hood;
  private Intake intake;
  private Indexer indexer;
  private boolean fin = false;

  public DiagnosticTest(Drivetrain drivetrain, Shooter shooter, Hood hood, Intake intake, Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hood = hood;
    this.intake = intake;
    this.indexer = indexer;
    addRequirements(drivetrain);
    addRequirements(shooter);
    addRequirements(hood);
    addRequirements(intake);
    addRequirements(indexer);
    fin = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateUser("Testing Drivetrain");
    drivetrain.runDrivetrain();
    updateUser("Testing right drivetrain");
    drivetrain.runDrivetrainright();
    updateUser("Testing left drivetrain");
    drivetrain.runDrivetrainleft();
    updateUser("Testing hood");
    hood.runHood();
    updateUser("Testing shooter");
    shooter.testShooter();
    updateUser("Testing intake");
    intake.runIntake();
    updateUser("Running indexer");
    indexer.runIndexer();
    fin = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }

  private void updateUser(String s) {
    SmartDashboard.putString("Diagnostic info", s);
    System.out.println(s);
  }
}
