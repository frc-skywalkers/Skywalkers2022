// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkMax climberMotorLeft = new CANSparkMax(ClimberConstants.kMotorLeftPort, MotorType.kBrushless);
  private final CANSparkMax climberMotorRight = new CANSparkMax(ClimberConstants.kMotorRightPort, MotorType.kBrushless);
  private final Servo climberServoLeftFirst = new Servo(ClimberConstants.kServoLeftFirstPort);
  private final Servo climberServoRightFirst = new Servo(ClimberConstants.kServoRightFirstPort);
  // private final Servo climberServoLeftSecond = new Servo(ClimberConstants.kServoLeftSecondPort);
  // private final Servo climberServoRightSecond = new Servo(ClimberConstants.kServoRightSecondPort);

  public Climber() {
    climberMotorLeft.restoreFactoryDefaults();
    climberMotorRight.restoreFactoryDefaults();
    climberMotorLeft.setInverted(ClimberConstants.kClimberLeftInvert);
    climberMotorRight.setInverted(ClimberConstants.kClimberRightInvert);

    latchFirst();
    // latchSecond();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rotateArms(double speed, boolean enableClimberArms) {
    if (!enableClimberArms || Math.abs(speed) < 0.075) {
      speed = 0;
    }

    climberMotorLeft.set(-speed);
    climberMotorRight.set(speed);
  }

  public void latchFirst() {
    climberServoLeftFirst.set(0.45);
    climberServoRightFirst.set(0);
  }

  public void unlatchFirst() {
    climberServoLeftFirst.set(0.01);
    climberServoRightFirst.set(0.525);
  }

  // public void latchSecond() {
  //   climberServoLeftSecond.set(0);
  //   climberServoLeftSecond.set(0.35);
  // }

  // public void unlatchSecond() {
  //   climberServoLeftSecond.set(0.35);
  //   climberServoRightSecond.set(0.05);
  // }
}
