// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private final CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.kHoodMotorPort, MotorType.kBrushless);
  private RelativeEncoder hoodEncoder;

  private final double HOOD_TOLERANCE = 1;
  private final double kP = 0.2;

  private double targetPosition = 0;
  private boolean stop = false;
  
  public Hood() {
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setInverted(ShooterConstants.kHoodInvert);
    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPosition(0);
    hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 150);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    hoodMotor.setSmartCurrentLimit(100, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double error = targetPosition - getPosition();
    // if (stop || Math.abs(error) < HOOD_TOLERANCE) {
    //   setOutput(0);
    // } else {
    //   setOutput(error * kP);
    // }

    SmartDashboard.putNumber("Hood Position", getPosition());
    SmartDashboard.putNumber("Hood Target", targetPosition);
    SmartDashboard.putNumber("Hood Voltage", hoodMotor.get());
  }

  public void setPosition(double target) {
    targetPosition = target;
    stop = false;
  }

  public void stop() {
    stop = true;
  }

  public void setOutput(double speed, boolean confirmation) {
    if (confirmation) {
      hoodMotor.set(MathUtil.clamp(speed, -0.4, 0.4));
    }
  }

  public void setOutput(double speed) {
    setOutput(speed, true);
  }

  public double getPosition() {
    return hoodEncoder.getPosition();
  }

  public boolean atPosition(double target, double tolerance) {
    return Math.abs(target - getPosition()) < tolerance;
  }

  public boolean atPosition(double tolerance) {
    return atPosition(targetPosition, tolerance);
  }
}
