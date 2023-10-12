// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule frontLeft =
      new SwerveModule(
          DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
          DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
          DriveConstants.FRONT_LEFT_DRIVE_ENCODER_PORTS,
          DriveConstants.FRONT_LEFT_TURNING_ENCODER_PORTS,
          DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
          DriveConstants.FRONT_LEFT_TURNING_ENCODER_REVERSED);

  private final SwerveModule rearLeft =
      new SwerveModule(
          DriveConstants.REAR_LEFT_DRIVE_MOTOR_PORT,
          DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
          DriveConstants.REAR_LEFT_DRIVE_ENCODER_PORTS,
          DriveConstants.REAR_LEFT_TURNING_ENCODER_PORTS,
          DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
          DriveConstants.REAR_LEFT_TURNING_ENCODER_REVERSED);

  private final SwerveModule frontRight =
      new SwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
          DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
          DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_PORTS,
          DriveConstants.FRONT_RIGHT_TURNING_ENCODER_PORTS,
          DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
          DriveConstants.FRONT_RIGHT_TURNING_ENCODER_REVERSED);

  private final SwerveModule rearRight =
      new SwerveModule(
          DriveConstants.REAR_RIGHT_DRIVE_MOTOR_PORT,
          DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
          DriveConstants.REAR_RIGHT_DRIVE_ENCODER_PORTS,
          DriveConstants.REAR_RIGHT_TURNING_ENCODER_PORTS,
          DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
          DriveConstants.REAR_RIGHT_TURNING_ENCODER_REVERSED);

  // The gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d(),
        frontLeft.getState(),
        rearLeft.getState(),
        frontRight.getState(),
        rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }
}
