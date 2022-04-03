// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(IntakeConstants.kIntakeInvert);
    intakeMotor.setSmartCurrentLimit(100, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Voltage", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Temperature", intakeMotor.getMotorTemperature());
  }

  public void intake() {
    intakeMotor.set(IntakeConstants.kMaxOutput);
  }

  public void stopRollers() {
    intakeMotor.stopMotor();
  }

  public void setRollerOutput(double speed) {
    intakeMotor.set(speed);
  }
}
