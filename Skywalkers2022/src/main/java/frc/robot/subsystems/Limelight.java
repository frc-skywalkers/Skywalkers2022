// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  private static final double CAM_MOUNTING_ANGLE = 35;
  private static final double CAM_HEIGHT = 27.5;
  private static final double GOAL_HEIGHT = 104;
  private static final double DISTANCE_OFFSET = 17;

  private NetworkTable table;
  private double x;
  private double y;
  private double distance;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(1);
    ledOff();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    updateValues();
  }

  public void updateValues() {
    x = table.getEntry("tx").getDouble(0);
    y = table.getEntry("ty").getDouble(0);
    distance = (GOAL_HEIGHT - CAM_HEIGHT) / Math.tan(Math.toRadians(CAM_MOUNTING_ANGLE + y));
    distance += DISTANCE_OFFSET;
    SmartDashboard.putNumber("Distance", distance);
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getDistance() {
    return distance;
  }

  public void ledOff() {
    table.getEntry("ledMode").setNumber(1);
  }

  public void ledOn() {
    table.getEntry("ledMode").setNumber(0);
  }
}
