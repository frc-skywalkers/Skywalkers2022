// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;

public class Funnel extends SubsystemBase {
  /** Creates a new Funnel. */
  CANSparkMax funnelMotor = new CANSparkMax(FunnelConstants.kMotorPort, MotorType.kBrushless);

  public Funnel() {
    funnelMotor.restoreFactoryDefaults();
    funnelMotor.setInverted(FunnelConstants.kInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutput(double speed) {
    funnelMotor.set(speed);
  }

  public void on() {
    setOutput(FunnelConstants.kFunnelSpeed);
  }

  public void off() {
    setOutput(0);
  }
}
