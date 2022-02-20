// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);

  public Arm() {
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(ArmConstants.kArmInvert);
  }

  public void arm(double speed) {
    armMotor.set(speed);
  }

  public void stop() {
    armMotor.stopMotor();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
