// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Shooter extends PIDSubsystem {
  /** Creates a new Shooter. */
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(ShooterConstants.kShooterMotorPortLeft);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(ShooterConstants.kShooterMotorPortRight);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    ShooterConstants.kStaticFriction, 
    ShooterConstants.kVelocity, 
    ShooterConstants.kAccel);


  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));

    leftMaster.configFactoryDefault();
    leftMaster.setInverted(ShooterConstants.kShooterInvert);
    leftMaster.setNeutralMode(NeutralMode.Coast);

    rightFollower.configFactoryDefault();
    rightFollower.follow(leftMaster);
    rightFollower.setInverted(InvertType.OpposeMaster);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMaster.configAllSettings(configs);
    rightFollower.configAllSettings(configs);

    
  }

  public void setVoltage(double volts) {
    leftMaster.setVoltage(volts); 
  }

  public void stopShoot() {
    leftMaster.setVoltage(0.000);
  }


  public double getSenPos() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getSenVel() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double getPosRotations() {
    return (double) getSenPos()/ShooterConstants.kUnitsPerRevolution;
  }

  public double getRotPerSec() {
    return (double) getSenVel()/ShooterConstants.kUnitsPerRevolution * 10;
  }

  public double getRotPerMin() {
    return (double) getSenVel() * 60.0;
  }

  
  @Override
  public void useOutput(double output, double setpoint) {
    double feedforward = m_feedforward.calculate(setpoint);
    setVoltage(output + feedforward);
    return;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getSenVel();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Shooter Velocity", getRotPerMin());
  }
}
