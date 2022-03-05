// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
  private final CANSparkMax armMotor = new CANSparkMax(IntakeConstants.kArmMotor, MotorType.kBrushless);

  private RelativeEncoder m_encoder;

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(IntakeConstants.kIntakeInvert);

    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(IntakeConstants.kArmInvert);

    m_encoder = armMotor.getEncoder();

    // Soft Limits <-- needs testing
    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) IntakeConstants.kMaxArmThreshold);

    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) IntakeConstants.kMinArmThreshold);

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public void intake() {
    intakeMotor.set(IntakeConstants.kMaxOutput);
  }

  public void stopRollers() {
    intakeMotor.stopMotor();
  }

  public void setArmOutput(double speed) {
    armMotor.set(speed);
  }

  public double getArmPosition() { 
    return m_encoder.getPosition();
  }

  public double getArmOutput() {
    return armMotor.getAppliedOutput();
  }


  public void setRollerOutput(double speed) {
    intakeMotor.set(speed);
  }

  public double getRollerOutput() {
    return intakeMotor.getAppliedOutput(); // need to test this
  }

  public boolean isDeployed() {
    if (getArmPosition() >= IntakeConstants.kArmThreshold) {
      return true;
    } else {
      return false;
    }
  }
}