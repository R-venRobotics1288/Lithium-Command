// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 0;
        public static final int REAR_LEFT_DRIVE_MOTOR_PORT = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 4;
        public static final int REAR_RIGHT_DRIVE_MOTOR_PORT = 6;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 1;
        public static final int REAR_LEFT_TURNING_MOTOR_PORT = 3;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
        public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 7;

        public static final int[] FRONT_LEFT_TURNING_ENCODER_PORTS = new int[]{0, 1};
        public static final int[] REAR_LEFT_TURNING_ENCODER_PORTS = new int[]{2, 3};
        public static final int[] FRONT_RIGHT_TURNING_ENCODER_PORTS = new int[]{4, 5};
        public static final int[] REAR_RIGHT_TURNING_ENCODER_PORTS = new int[]{6, 7};

        public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = false;
        public static final boolean REAR_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = false;
        public static final boolean REAR_RIGHT_TURNING_ENCODER_REVERSED = true;

        public static final int[] FRONT_LEFT_DRIVE_ENCODER_PORTS = new int[]{8, 9};
        public static final int[] REAR_LEFT_DRIVE_ENCODER_PORTS = new int[]{10, 11};
        public static final int[] FRONT_RIGHT_DRIVE_ENCODER_PORTS = new int[]{12, 13};
        public static final int[] REAR_RIGHT_DRIVE_ENCODER_PORTS = new int[]{14, 15};

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean REAR_LEFT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean REAR_RIGHT_DRIVE_ENCODER_REVERSED = true;

        public static final double TRACK_WIDTH = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = 0.7;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        public static final boolean GYRO_REVERSED = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    }

    public static final class ModuleConstants {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

        public static final int ENCODER_CPR = 1024;
        public static final double WHEEL_DIAMETER_METERS = 0.15;
        public static final double DRIVE_ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double TURNING_ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double P_MODULE_TURNING_CONTROLLER = 1;

        public static final double P_MODULE_DRIVE_CONTROLLER = 1;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }
}
