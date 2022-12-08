// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class swerveModConstants {
        public static final double kTurningP = 0.5;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;

        public static final double kCPR = 0.0048289738430584;

        // Front left module
        public static final class mod1 {
            public static final int turningMotorID = 1;
            public static final int drivingMotorID = 8;
            public static final int[] relEncoderID = {9,8};
        }

        // Back left module
        public static final class mod2 {
            public static final int turningMotorID = 2;
            public static final int drivingMotorID = 7;
            public static final int[] relEncoderID = {2,3};
        }

        // Front right module
        public static final class mod3 {
            public static final int turningMotorID = 3;
            public static final int drivingMotorID = 6;
            public static final int[] relEncoderID = {0,1};
        }

        // Back right module
        public static final class mod4 {
            public static final int turningMotorID = 4;
            public static final int drivingMotorID = 5;
            public static final int[] relEncoderID = {7,6};
        }

    public static final class driveConstants {
        public static final double kWheelBase = 24;
        public static final double kTrack = 24;

        public static final double kSpeedMultiplier = 1;

        public static final double kMaxSpeedMPS = 5;
    }

    }
}
