// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkMax climberMotorLeft = new CANSparkMax(ClimberConstants.kMotorLeftPort, MotorType.kBrushless);
  private final CANSparkMax climberMotorRight = new CANSparkMax(ClimberConstants.kMotorRightPort, MotorType.kBrushless);

  public Climber() {
    climberMotorLeft.restoreFactoryDefaults();
    climberMotorRight.restoreFactoryDefaults();
    climberMotorLeft.setInverted(ClimberConstants.kClimberLeftInvert);
    climberMotorRight.setInverted(ClimberConstants.kClimberRightInvert);
    climberMotorLeft.setSmartCurrentLimit(181, 2);
    climberMotorRight.setSmartCurrentLimit(181, 2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Voltage", climberMotorLeft.get());
  }

  public void rotateArms(double speed, boolean enableClimberArms) {
    if (!enableClimberArms || Math.abs(speed) < OIConstants.kDeadZone) {
      speed = 0;
    }

    climberMotorLeft.set(-speed);
    climberMotorRight.set(speed);
  }
}
