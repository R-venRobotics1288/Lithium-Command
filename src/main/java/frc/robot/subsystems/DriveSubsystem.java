// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
	// Robot swerve modules

	private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(DriveConstants.SPEED_RATE_LIMIT);
	private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(DriveConstants.SPEED_RATE_LIMIT);
	private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(DriveConstants.SPEED_ROT_LIMIT);

	public final SwerveModule frontLeft =
			new SwerveModule(
					DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
					DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
							DriveConstants.ENCODER_OFFSETS[0], true);

	public final SwerveModule rearLeft =
			new SwerveModule(
					DriveConstants.REAR_LEFT_DRIVE_MOTOR_PORT,
					DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
							DriveConstants.ENCODER_OFFSETS[1], true);

	public final SwerveModule frontRight =
			new SwerveModule(
					DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
					DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
							DriveConstants.ENCODER_OFFSETS[2], true);

	public final SwerveModule rearRight =
			new SwerveModule(
					DriveConstants.REAR_RIGHT_DRIVE_MOTOR_PORT,
					DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
							DriveConstants.ENCODER_OFFSETS[3], true);

	// The gyro sensor
	private final PigeonIMU gyro = new PigeonIMU(30);

	// Odometry class for tracking robot pose
	SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(gyro.getYaw()),
		new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearLeft.getPosition(),
			rearRight.getPosition()
		}
	);

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		odometry.update(
				new Rotation2d(gyro.getYaw()),
				new SwerveModulePosition[] {
								frontLeft.getPosition(),
								frontRight.getPosition(),
								rearLeft.getPosition(),
								rearRight.getPosition()
				});
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
		odometry.resetPosition(new Rotation2d(gyro.getYaw()),
						new SwerveModulePosition[] {
										frontLeft.getPosition(),
										frontRight.getPosition(),
										rearLeft.getPosition(),
										rearRight.getPosition()
		}, pose);
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
		//var xSpeed1 = -xSpeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer
					 // .driverController.getLeftY(), Constants.ModuleConstants.DEADBAND));

		//var ySpeed1 = -ySpeedLimiter.calculate(MathUtil
//.applyDeadband(RobotContainer.driverController.getLeftX(), Constants.ModuleConstants.DEADBAND));

		SwerveModuleState[] swerveModuleStates =
				DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
						fieldRelative
								? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(getGyroValue()))
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
		gyro.setYaw(0);
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return gyro.getYaw();
	}

	private double getGyroValue() {
		return gyro.getYaw() * Math.PI / 180;
	}


	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
/*  public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
	}*/

	public double getFrontLeft() {
		return frontLeft.getValue();
	}

	public double getRearLeft() {
		return rearLeft.getValue();
	}

	public double getFrontRight() {
		return frontRight.getValue();
	}

	public double getRearRight() {
		return rearRight.getValue();
	}

	public void stop()
	{
		frontLeft.stop();
		frontRight.stop();
		rearLeft.stop();
		rearRight.stop();
	}
}

