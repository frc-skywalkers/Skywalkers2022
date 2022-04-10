// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static final int kDriverController1Port = 0;
        public static final int kDriverController2Port = 1;

        public static final int kLeftX = 0;
        public static final int kLeftY = 1;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;
        public static final int kRightX = 4;
        public static final int kRightY = 5;

        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;

        public static final double kDeadZone = 0.1;
    }

    public static final class DriveConstants {
        public static final int kLeftMasterPort = 3;
        public static final int kLeftFollowerPort = 4;
        public static final int kRightMasterPort = 1;
        public static final int kRightFollowerPort = 2;

        public static final TalonFXInvertType kLeftInvertType = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType kRightInvertType = TalonFXInvertType.CounterClockwise;

        public static final double kMaxOutput = 0.85;
        public static final double kSlowOutput = 0.35;
        public static final double kTurnOutput = 0.7;

        public static final double kGearRatio = 11.0/60.0 * 16.0/32.0;
        public static final double kTicksPerRotation = 2048;
        public static final double kWheelDiameter = Units.inchesToMeters(6.0);
        public static final double kDistancePerPulseFactor = kWheelDiameter * Math.PI * kGearRatio / 2048;

        public static final double kTiltThreshold = 5;
        public static final double kTiltP = 0.05;
    }

    public static final class ClimberConstants {
        public static final int kMotorLeftPort = 5;
        public static final int kMotorRightPort = 14;
        public static final boolean kClimberLeftInvert = false;
        public static final boolean kClimberRightInvert = false;
    }
    
    public static final class IntakeConstants {
        public static final int kIntakeMotor = 8;
        public static final double kMaxOutput = 0.5;
        public static final boolean kIntakeInvert = true;
    }

    public static final class ArmConstants {
        public static final int kMotorPort = 23;
        public static final double kMaxOutput = 0.3;
        public static final boolean kArmInvert = true;
    }

    public static final class ShooterConstants {
        public static final int kShooterMotorPortLeft = 20;
        public static final int kShooterMotorPortRight = 21;
        public static final int kHoodMotorPort = 22;

        public static final boolean kShooterInvert = false;
        public static final boolean kHoodInvert = false;
        public static final double kDistancePerPulse = Units.inchesToMeters(4) * Math.PI / 2048;
    }

    public static final class IndexerConstants {
        public static int kMotorPort = 25;
        public static boolean kInvert = true;
        public static double kIndexerSpeed = 0.4;
    }
}
