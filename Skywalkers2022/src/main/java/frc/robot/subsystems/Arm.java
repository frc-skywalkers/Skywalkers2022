// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private RelativeEncoder armEncoder;

  public Arm() {
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(ArmConstants.kArmInvert);
    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);
    
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, 13);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
  }

  public void arm(double speed) {
    armMotor.set(speed);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public double getPosition() {
    return armEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", getPosition());
  }
}
