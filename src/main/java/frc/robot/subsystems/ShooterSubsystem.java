package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterLeft;
  private CANSparkMax shooterRight;
  private CANSparkMax feederMotor;

  Pose3d robotPositonToApril;

  public ShooterSubsystem(CANSparkMax feederMotor) {
    this.feederMotor = feederMotor;
    shooterLeft = new CANSparkMax(DriveConstants.LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    shooterRight = new CANSparkMax(DriveConstants.RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    // robotPositonToApril = new Pose3d();

    shooterLeft.setSmartCurrentLimit(80);
    shooterRight.setSmartCurrentLimit(80);
  }

  // @Override
  // public void periodic() {
  //     checkAprilTag();
  // }

  // private Command checkAprilTag() {
  //     return this.runOnce(() -> robotPositonToApril =
  // cameraSubsystem.getLastEstimatedRobotPose(false));
  // }

  // public Command cameraAutoAlign() {
  //     return this.run(() -> {
  //         if (gcontroller.getRawButton(1)) {
  //             System.out.println("Hi");
  //         }
  //     });

  // }

  public Command FeederForward() {
    return this.runOnce(
        () -> {
          feederMotor.set(ShooterConstants.FEEDER_FORWARD_SPEED);
        });
  }

  public Command FeederReverse() {
    return this.runOnce(
        () -> {
          feederMotor.set(-ShooterConstants.FEEDER_REVERSE_SPEED);
        });
  }

  public Command FeederReverseSlow() {
    return this.runOnce(
        () -> {
          feederMotor.set(-0.15);
        });
  }

  public Command FeederStop() {
    return this.runOnce(
        () -> {
          feederMotor.set(0);
        });
  }

  public Command ShooterReverse() {
    return this.runOnce(
        () -> {
          shooterLeft.set(-ShooterConstants.SHOOTER_REVERSE_SPEED);
          shooterRight.set(ShooterConstants.SHOOTER_REVERSE_SPEED);
        });
  }

  public Command ShooterForwardSpeaker() {
    return this.runOnce(
        () -> {
          System.out.println("Running");
          shooterLeft.set(ShooterConstants.SHOOTER_SPEAKER_SPEED);
          shooterRight.set(-ShooterConstants.SHOOTER_SPEAKER_SPEED);
        });
  }

  public Command ShooterForwardAmp() {
    return this.runOnce(
        () -> {
          shooterLeft.set(ShooterConstants.SHOOTER_AMP_SPEED);
          shooterRight.set(-ShooterConstants.SHOOTER_AMP_SPEED);
        });
  }

  public Command ShooterStop() {
    return this.runOnce(
        () -> {
          System.out.println("Shooter Stop");
          shooterLeft.set(0);
          shooterRight.set(0);
        });
  }
}
