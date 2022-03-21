package frc.robot;

import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import org.ejml.simple.SimpleMatrix;

public class MathFunctions {
  // All units are in inches and radians
  private static final double HIGH_GOAL_RADIUS = 24;
  private static final double HIGH_GOAL_X = 1; // TODO: fix based on field coordinate system
  private static final double HIGH_GOAL_Y = 1; // TODO: fix based on field coordinate system
  private static final double HIGH_GOAL_HEIGHT = 104;
  private static final double SHOOTER_HEIGHT = 26.6;
  private static final double g = 385.8;

  public MathFunctions() {}

  public static double[] calculateCoefficients(double robotX, double robotY, double shooterError) {
    double d = hypot(robotX - HIGH_GOAL_X, robotY - HIGH_GOAL_Y);

    SimpleMatrix A = new SimpleMatrix(new double[][] {{0, 0, 1}, {(d - 24) * (d - 24), d - 24, 1}, {d * d, d, 1}});
    SimpleMatrix b = new SimpleMatrix(new double[][] {{SHOOTER_HEIGHT}, {109 + shooterError}, {104}});
    SimpleMatrix x = A.solve(b);

    double theta = atan(x.get(1, 0));
    double v = sqrt(-g / (2*x.get(0, 0))) / cos(theta);

    return new double[] {theta, v};
  }

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
