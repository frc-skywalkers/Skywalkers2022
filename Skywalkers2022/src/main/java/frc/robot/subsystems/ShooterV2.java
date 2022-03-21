// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class ShooterV2 extends SubsystemBase {
  /** Creates a new ShooterV2. */

  private final WPI_TalonFX leftMaster = new WPI_TalonFX(ShooterConstants.kShooterMotorPortLeft);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(ShooterConstants.kShooterMotorPortRight);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 12/32.8, 0);

  private BangBangController controller = new BangBangController();

  public ShooterV2() {
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
    controller.setTolerance(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getRPS());
  }

  // public void setSpeed(double rps) {
  //   System.out.println("Called");
  //   double feedforward = m_feedforward.calculate(rps);
  //   System.out.println("Setting voltage to:" + controller.calculate(getRPS(), rps) * 6);
  //   System.out.println(rps);
  //   setVoltage(controller.calculate(getRPS(), rps) * 6);
  // }

  public void setSpeed(double rps) {
    double error = rps - getRPS();
    double feedforward = m_feedforward.calculate(rps);
    setVoltage(error * 0.05 + feedforward);
  }

  public void setVoltage(double volts) {
    leftMaster.setVoltage(volts); 
  }

  public void stopShoot() {
    leftMaster.setVoltage(0.000);
  }

  public boolean atSpeed(double targetSpeed, double tolerance) {
    if (Math.abs(getRPS() - targetSpeed) < tolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atSpeed( double tolerance) {
    if (Math.abs(getRPS() - RobotContainer.ranges[RobotContainer.rangeIndex][1]) < tolerance) {
      return true;
    } else {
      return false;
    }
  }


  public double getPos() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getVel() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double getRotations() {
    return (double) getPos() * ShooterConstants.kDistancePerPulse;
  }

  public double getRPS() {
    return (double) getVel() * ShooterConstants.kDistancePerPulse * 10;
  }

  public double getRPM() {
    return (double) getVel() * 60.0;
  }
}
