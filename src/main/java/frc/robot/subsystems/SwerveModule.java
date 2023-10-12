// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SwerveModule {
  private final Spark driveMotor;
  private final Spark turningMotor;

  private final Encoder driveEncoder;
  private final Encoder turningEncoder;

  private final PIDController drivePIDController =
      new PIDController(ModuleConstants.P_MODULE_DRIVE_CONTROLLER, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.P_MODULE_TURNING_CONTROLLER,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND,
              ModuleConstants.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderPorts,
      int[] turningEncoderPorts,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    driveMotor = new Spark(driveMotorChannel);
    turningMotor = new Spark(turningMotorChannel);

    this.driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

    this.turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setDistancePerPulse(ModuleConstants.DRIVE_ENCODER_DISTANCE_PER_PULSE);

    // Set whether drive encoder should be reversed or not
    driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    turningEncoder.setDistancePerPulse(ModuleConstants.TURNING_ENCODER_DISTANCE_PER_PULSE);

    // Set whether turning encoder should be reversed or not
    turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getRate(), new Rotation2d(turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.get()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        turningPIDController.calculate(turningEncoder.get(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    turningMotor.set(turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.reset();
    turningEncoder.reset();
  }
}
