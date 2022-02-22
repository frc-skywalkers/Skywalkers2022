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
  private final Servo climberServoLeftSecond = new Servo(ClimberConstants.kServoLeftSecondPort);
  private final Servo climberServoRightSecond = new Servo(ClimberConstants.kServoRightSecondPort);

  private double speedControl = ClimberConstants.kInitialSpeedControl;
  private int motorSelection = -1;

  public Climber() {
    climberMotorLeft.restoreFactoryDefaults();
    climberMotorRight.restoreFactoryDefaults();
    climberMotorLeft.setInverted(ClimberConstants.kClimberLeftInvert);
    climberMotorRight.setInverted(ClimberConstants.kClimberRightInvert);

    latchFirst();
    latchSecond();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Jank method used only for debugging
  public void move(boolean a, boolean b, boolean x, boolean y, boolean leftBumper, boolean rightBumper, double speed, int dpad) {
    // Print for debug
    System.out.println(a + " " + b + " " + x + " " + y + " " + leftBumper + " " + rightBumper + " " + speed + " " + dpad + " " + speedControl + " " + System.currentTimeMillis());

    // Increase motor speed sensitivity
    if (dpad == 0) {
      speedControl = Math.min(1, speedControl + 0.01);
    }

    // Decrease motor speed sensitivity
    if (dpad == 180) {
      speedControl = Math.max(-1, speedControl - 0.01);
    }

    // First rung latch/unlatch
    if (leftBumper) {
      unlatchFirst();
    } else {
      latchFirst();
    }

    // Second rung latch/unlatch
    if (rightBumper) {
      unlatchSecond();
    } else {
      latchSecond();
    }

    // Speed deadzone
    if (Math.abs(speed) < 0.075) {
      speed = 0;
    }

    // Select which motors to run (None, Left, Both, Right)
    if (a) {
      motorSelection = -1;
    } else if (x) {
      motorSelection = 0;
    } else if (y) {
      motorSelection = 1;
    } else if (b) {
      motorSelection = 2;
    }

    // Run motors appropriately
    if (motorSelection == 0) {
      climberMotorLeft.set(speedControl * -speed);
    } else if (motorSelection == 1) {
      climberMotorLeft.set(speedControl * -speed);
      climberMotorRight.set(speedControl * speed);
    } else if (motorSelection == 2) {
      climberMotorRight.set(speedControl * speed);
    } else {
      climberMotorLeft.set(0);
      climberMotorRight.set(0);
    }
  }

  public void rotateArms(double speed) {
    if (Math.abs(speed) < 0.075) {
      speed = 0;
    }

    climberMotorLeft.set(-speed);
    climberMotorRight.set(speed);
  }

  public void latchFirst() {
    climberServoLeftFirst.set(0.35);
    climberServoRightFirst.set(0);
  }

  public void unlatchFirst() {
    climberServoLeftFirst.set(0.05);
    climberServoRightFirst.set(0.35);
  }

  public void latchSecond() {
    climberServoLeftSecond.set(0);
    climberServoLeftSecond.set(0.35);
  }

  public void unlatchSecond() {
    climberServoLeftSecond.set(0.35);
    climberServoRightSecond.set(0.05);
  }
}
