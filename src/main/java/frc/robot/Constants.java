// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final boolean isReplay = false;

    public static class ControllerConstants {
        // Controller IDs
        public static final int driverId = 0;
        public static final int operatorId = 1;

        // Controller deadband
        public static final double deadband = 0.1; // Percent
    }

    public static class DriveConstants {
        // Moment of Inertia
        public static final double batteryWeight = Units.lbsToKilograms(12.5);
        public static final double batteryToCenter = Units.inchesToMeters(10);
        public static final double gearboxWeight = Units.lbsToKilograms(2.8 * 2.0) + 2.0;
        public static final double gearboxToCenter = Units.inchesToMeters(13);
        public static final double momentOfInertia = (batteryWeight * Math.pow(batteryToCenter, 2)) + (gearboxWeight * Math.pow(gearboxToCenter, 2));

        // Motor IDs
        public static final int frontLeftID = 1;
        public static final int backLeftID = 2;
        public static final int frontRightID = 3;
        public static final int backRightID = 4;

        // Robot specs
        public static final double robotWidth = Units.inchesToMeters(13); // Meters
        public static final double wheelRadius = Units.inchesToMeters(4); // Meters
        public static final double wheelCircumference = wheelRadius * 2.0 * Math.PI; // Meters
        public static final double gearRatio = 8.46;
        public static final double mass = Units.lbsToKilograms(60);

        // Max speeds
        public static final double maxClosedDriveSpeed = 3.5; // Meters / Second
        public static final double maxClosedTurnSpeed = maxClosedDriveSpeed / (robotWidth / 2); // Meters / Second
        public static final double maxOpenDriveSpeed = 1; // Percent
        public static final double maxOpenTurnSpeed = 0.7; // Percent

        // Conversion factors
        public static final double rotToMeters = wheelCircumference / gearRatio;

        // PIDFF values
        public static final double kP   = 0.0;
        public static final double kI   = 0.0;
        public static final double kD   = 0.0;
        public static final double kFF  = 0.0;
    }

    public static class ShooterConstants {
        // Motor IDs
        public static final int feedID = 11;
        public static final int launchID = 12;

        // Max speeds
        public static final double maxFeedSpeed = 5676; // Rotations / Minute
        public static final double maxLaunchSpeed = 5676; // Rotations / Minute

        // PIDFF values
        public static final double launchP  = 0.0;
        public static final double launchI  = 0.0;
        public static final double launchD  = 0.0;
        public static final double launchFF = 0.0;
        public static final double feedP    = 0.0;
        public static final double feedI    = 0.0;
        public static final double feedD    = 0.0;
        public static final double feedFF   = 0.0;
    }
}