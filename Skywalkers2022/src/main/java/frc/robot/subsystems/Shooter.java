// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final WPI_TalonFX leftMaster = new WPI_TalonFX(ShooterConstants.kShooterMotorPortLeft);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(ShooterConstants.kShooterMotorPortRight);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 12/32.8, 0);

  private double targetRPS = 0;
  private boolean stop = false;

  public Shooter() {
    leftMaster.configFactoryDefault();
    leftMaster.setInverted(ShooterConstants.kShooterInvert);
    leftMaster.setNeutralMode(NeutralMode.Coast);

    rightFollower.configFactoryDefault();
    rightFollower.follow(leftMaster);
    rightFollower.setInverted(InvertType.OpposeMaster);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    configs.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 240, 255, 5);

    leftMaster.configAllSettings(configs);
    rightFollower.configAllSettings(configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (stop) {
      setVoltage(0);
    } else {
      double error = targetRPS - getRPS();
      double feedforward = m_feedforward.calculate(targetRPS);
      setVoltage(error * 0.05 + feedforward);
    }

    SmartDashboard.putNumber("Shooter Velocity", getRPS());
    SmartDashboard.putNumber("Shooter Target", targetRPS);
    SmartDashboard.putNumber("Shooter Voltage", leftMaster.get());
  }

  public void setSpeed(double rps) {
    targetRPS = rps;
    stop = false;
  }

  public void stopShoot() {
    stop = true;
  }

  public void setVoltage(double volts) {
    leftMaster.setVoltage(volts); 
  }

  public boolean atSpeed(double targetSpeed, double tolerance) {
    return Math.abs(targetSpeed - getRPS()) < tolerance;
  }

  public boolean atSpeed(double tolerance) {
    return atSpeed(targetRPS, tolerance);
  }

  public double getVel() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double getRPS() {
    return getVel() * ShooterConstants.kDistancePerPulse * 10;
  }
}
