// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkMax climberMotor = new CANSparkMax(ClimberConstants.kMotorPort, MotorType.kBrushless);
  private double speedControl = ClimberConstants.kInitialSpeedControl;

  public Climber() {
    climberMotor.restoreFactoryDefaults();
    climberMotor.setInverted(ClimberConstants.kClimberInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed, double speedIncrement) {
    speedControl += 0.01 * speedIncrement;
    climberMotor.set(speedControl * speed);
  }

  public void stop() {
    climberMotor.set(0);
  }
}
