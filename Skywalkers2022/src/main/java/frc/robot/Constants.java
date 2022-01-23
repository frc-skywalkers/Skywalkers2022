// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

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
        public static final int kDriverControllerPort = 1;
        public static final int kLeftY = 1;
        public static final int kRightX = 4;
        public static final int kRightY = 5;
    }

    public static final class DriveConstants {
        public static final int kLeftMasterPort = 3;
        public static final int kLeftFollowerPort = 4;
        public static final int kRightMasterPort = 1;
        public static final int kRightFollowerPort = 2;

        public static final TalonFXInvertType kLeftInvertType = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType kRightInvertType = TalonFXInvertType.Clockwise;

        public static final double kMaxOutput = 0.3;
    }

    public static final class ClimberConstants {
        public static final int kMotorPort = 0; // might need to be changed
        
        public static final boolean kClimberInvert = false; // might need to be changed
    }
}
