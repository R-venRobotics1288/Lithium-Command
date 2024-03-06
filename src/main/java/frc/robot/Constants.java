// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;

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

        /**
         * The Order from top to bottom is:
         * Front Left,
         * Rear Left,
         * Front Right,
         * Rear Right
         * */
        // public static final double[] ENCODER_OFFSETS = {
        //         0.0006103515625,
        //         0.0001220703125,
        //         0.0010986328125,
        //         0.00048828125
        // };

        public static final double[] ENCODER_OFFSETS = {
                Math.PI - Math.toRadians(0.73),
                Math.PI - Math.toRadians(357.18),
                Math.PI - Math.toRadians(0.28),
                Math.PI - Math.toRadians(359.32)
        };

        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 4;
        public static final int REAR_LEFT_DRIVE_MOTOR_PORT = 5;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;
        public static final int REAR_RIGHT_DRIVE_MOTOR_PORT = 10;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 7;
        public static final int REAR_LEFT_TURNING_MOTOR_PORT = 1;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 6;
        public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 9;

        public static final int INTAKE_MOTOR_PORT = 11;
        public static final int POSITION_MOTOR_PORT = 12;
        public static final int FEEDER_MOTOR_PORT = 13;

        public static final int LEFT_SHOOTER_MOTOR_PORT = 14;
        public static final int RIGHT_SHOOTER_MOTOR_PORT = 15;

        public static final int LEFT_ELEVATOR_MOTOR_PORT = 16;
        public static final int RIGHT_ELEVATOR_MOTOR_PORT = 17;

        public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = false;
        public static final boolean REAR_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = false;
        public static final boolean REAR_RIGHT_TURNING_ENCODER_REVERSED = true;

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean REAR_LEFT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean REAR_RIGHT_DRIVE_ENCODER_REVERSED = true;

        public static final double SPEED_RATE_LIMIT = 1.0;
        public static final double SPEED_ROT_LIMIT = 3.0;

        public static final double TRACK_WIDTH = 25.5625;
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = 26.0;
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

        public static final double MAX_SPEED_METERS_PER_SECOND = 10;
    }


    public static final class ModuleConstants {
        // Name of camera used by the CameraSubsystem
        public static final String CAMERA_NAME = "raven1288";

        // I2C port of the REV Colour Sensor V3 used by ColourSensorSubsystem
        public static final I2C.Port COLOUR_SENSOR_PORT = I2C.Port.kMXP;

        // DIO address of all Limit Switches we intend to use, add to here if you want to add a switch.
        public static final int[] LIMIT_SWITCHES = new int[] {
                0
        };

        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

        public static final double MAX_MODULE_METERS_PER_SECOND = 10;
      
        // public static final double GEAR_RATIO = 8.14;
  
        public static final int ENCODER_CPR = 8192;
        public static final double WHEEL_DIAMETER_METERS = 0.076;
        public static final double DRIVE_ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double TURNING_ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double P_MODULE_TURNING_CONTROLLER = 0.4;

        public static final double P_MODULE_DRIVE_CONTROLLER = 0.1;

        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);

        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelCircumferenceMeters = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / DRIVING_MOTOR_REDUCTION; // Current value 4.7902941315418912960821043628581

        public static final double kDrivingEncoderPositionFactor = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION; // meters
        public static final double kDrivingEncoderVelocityFactor = ((WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI; // radians
                
        public static final double DEADBAND = 0.4;

    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class AutoConstants 
    {
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

    public static final class NeoMotorConstants 
    {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class ShooterConstants
    {
        public static final double FEEDER_SPEED = 0.15;
        public static final double SHOOTER_REVERSE_SPEED = 0.15;
        public static final double SPEAKER_SPEED = 0.5;
        public static final double AMP_SPEED = 0.4;
    }
    public static final class ElevatorConstants
    {
        public static final double TOP_ELEVATOR_LIMIT = 24;
        public static final double BOT_ELEVATOR_LIMIT = 0;

        public static final double LEFT_MOTOR_SPEED = 0.1;
        public static final double RIGHT_MOTOR_SPEED = 0.1;
    }
}
