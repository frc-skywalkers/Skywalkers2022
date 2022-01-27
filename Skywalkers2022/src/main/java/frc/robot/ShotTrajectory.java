package frc.robot;

import edu.wpi.first.math.util.Units;

public class ShotTrajectory {

    // Robot Constants
    private static final double shooterHeight = Units.inchesToMeters(30); // TBD
    private static final double maxAngularVelocity = Units.rotationsPerMinuteToRadiansPerSecond(6000); // TBD
    private static final double minAngle = Units.degreesToRadians(45); // TBD
    private static final double maxAngle = Units.degreesToRadians(90);
    private static final double momentOfInertia = -1; // TBD

    // Goal Constants
    private static final double goalHeight = Units.inchesToMeters(8 * 12 + 12);
    private static final double goalX = Units.inchesToMeters(324);
    private static final double goalY = Units.inchesToMeters(162);

    // Pose
    private double x_pos;
    private double y_pos;

    // Shot 
    private double theta_0;
    private double v_0;
    private double w_0;

    // Phyiscs Constants
    private double g = 9.8;


    public ShotTrajectory(double x, double y) {
        this.x_pos = x;
        this.y_pos = y;
        this.calculateTrajectory();
    }

    public void calculateTrajectory() {

    }

    public void calculateMinVelocityTrajectory() {

    }

    public void calculateEntryAngle(double v, double theta) {

    }

    public void calculateEntryAngleTrajectory(double theta) {

    }

    public void linearToAngularVelocity() {

    }

    public double getInitialAngle() {
        return this.theta_0;
    }

    public double getLinearVelocity(){
        return this.v_0;
    }

    public double getAngularVelocity(){
        return this.w_0;
    }
}
