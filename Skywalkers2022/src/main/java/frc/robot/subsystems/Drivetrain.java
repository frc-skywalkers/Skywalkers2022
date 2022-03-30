// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private final WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.kLeftMasterPort);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(DriveConstants.kLeftFollowerPort);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.kRightMasterPort);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(DriveConstants.kRightFollowerPort);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(leftMaster, leftFollower);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(rightMaster, rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private final PigeonIMU imu = null;//new PigeonIMU(DriveConstants.PigeonIMUPort);

  public Drivetrain() {
    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMaster.configAllSettings(configs);
    leftFollower.configAllSettings(configs);
    rightMaster.configAllSettings(configs);
    rightFollower.configAllSettings(configs);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    leftMaster.setInverted(DriveConstants.kLeftInvertType);
    leftFollower.setInverted(DriveConstants.kLeftInvertType);
    rightMaster.setInverted(DriveConstants.kRightInvertType);
    rightFollower.setInverted(DriveConstants.kRightInvertType);

    drive.setMaxOutput(DriveConstants.kMaxOutput);

    // resetHeading();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    drive.stopMotor();
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
  }

  public double getLeftEncoderDistance() {
    return leftMaster.getSelectedSensorPosition() * DriveConstants.kDistancePerPulseFactor;
  }

  public double getRightEncoderDistance() {
    return rightMaster.getSelectedSensorPosition() * DriveConstants.kDistancePerPulseFactor;
  }

  public double getRightEncoderRate() {
      return rightMaster.getSelectedSensorVelocity() * DriveConstants.kDistancePerPulseFactor / 60;
  }

  public double getLeftEncoderRate() {
      return leftMaster.getSelectedSensorVelocity() * DriveConstants.kDistancePerPulseFactor / 60;
  }
  
  public void resetDrivetrainEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void resetHeading() {
    imu.setYaw(0);
  }

  public double getHeading() {
    double[] YPR = new double[3];
    imu.getYawPitchRoll(YPR);
    return YPR[0];
  }
}
