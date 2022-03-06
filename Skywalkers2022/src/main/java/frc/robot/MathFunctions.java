package frc.robot;

import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

public class MathFunctions {
  // All units are in inches and radians
  private static final double HIGH_GOAL_RADIUS = 24;
  private static final double HIGH_GOAL_X = 1; // TODO: fix based on field coordinate system
  private static final double HIGH_GOAL_Y = 1; // TODO: fix based on field coordinate system
  private static final double HIGH_GOAL_HEIGHT = 104;
  private static final double SHOOTER_HEIGHT = 30;
  private static final double g = 385.8;

  public MathFunctions() {}

  public static double calculateHoodAngle(double robotX, double robotY, double distanceOffset) {
    double d = hypot(robotX - HIGH_GOAL_X, robotY - HIGH_GOAL_Y) - distanceOffset;
    double height = HIGH_GOAL_HEIGHT - SHOOTER_HEIGHT;
    return atan((hypot(d, height) + height) / d);
  }

  public static double calculateFlywheelVelocity(double robotX, double robotY, double hoodAngle) {
    double d = hypot(robotX - HIGH_GOAL_X, robotY - HIGH_GOAL_Y);
    double height = HIGH_GOAL_HEIGHT - SHOOTER_HEIGHT;
    return sqrt((g * d * d) / (2 * (d * tan(hoodAngle) - height) * cos(hoodAngle) * cos(hoodAngle)));
  }

  public static double[] calculateShooterControlVariables(double robotX, double robotY) {
    double hoodAngleLower = calculateHoodAngle(robotX, robotY, -HIGH_GOAL_RADIUS);
    double hoodAngleUpper = calculateHoodAngle(robotX, robotY, HIGH_GOAL_RADIUS);
    double hoodAngle = (hoodAngleLower + hoodAngleUpper) / 2;
    double flywheelVelocity = calculateFlywheelVelocity(robotX, robotY, hoodAngle);
    return new double[] {hoodAngle, flywheelVelocity};
  }
}
