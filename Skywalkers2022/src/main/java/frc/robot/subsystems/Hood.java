// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  private final CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.kHoodMotorPort, MotorType.kBrushless);
  private RelativeEncoder hoodEncoder;
  
  public Hood() {
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setInverted(ShooterConstants.kHoodInvert);
    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPosition(0);
    hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 150);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Position", getPosition());
  }

  public double getPosition() {
    return hoodEncoder.getPosition();
  }

  public void setOutput(double speed) {
    hoodMotor.set(MathUtil.clamp(speed, -0.4, 0.4));
  }

  public void setPosition(double target) {
    double error = target - getPosition();
    setOutput(error * 0.4);
  }

  public boolean atPosition(double target, double tolerance) {
    if (Math.abs(target - getPosition()) < tolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atPosition( double tolerance) {
    if (Math.abs(RobotContainer.ranges[RobotContainer.rangeIndex][0] - getPosition()) < tolerance) {
      return true;
    } else {
      return false;
    }
  }

}
