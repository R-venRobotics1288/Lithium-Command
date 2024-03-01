package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax feederMotor;

    private final XboxController gcontroller;

    Pose3d robotPositonToApril;

    public ShooterSubsystem(XboxController controller) {
        feederMotor = new CANSparkMax(DriveConstants.FEEDER_MOTOR_PORT,
        MotorType.kBrushless);
        shooterLeft = new CANSparkMax(DriveConstants.LEFT_SHOOTER_MOTOR_PORT,
        MotorType.kBrushless);
        shooterRight = new CANSparkMax(DriveConstants.RIGHT_SHOOTER_MOTOR_PORT,
        MotorType.kBrushless);
        gcontroller = controller;
        
        // robotPositonToApril = new Pose3d();

        shooterLeft.setSmartCurrentLimit(80);
        shooterRight.setSmartCurrentLimit(80);
        feederMotor.setSmartCurrentLimit(80);

        feederMotor.setIdleMode(IdleMode.kBrake);

        shooterLeft.burnFlash();
        shooterRight.burnFlash();
        feederMotor.burnFlash();
    }

    // @Override
    // public void periodic() {
    //     checkAprilTag();
    // }

    // private Command checkAprilTag() {
    //     return this.runOnce(() -> robotPositonToApril = cameraSubsystem.getLastEstimatedRobotPose(false));
    // }

    // public Command cameraAutoAlign() {
    //     return this.run(() -> {
    //         if (gcontroller.getRawButton(1)) {
    //             System.out.println("Hi");
    //         }
    //     });

    // }

    // TODO: event driven button input
    public void buttonShoot() {
        // Feeder Motor
        if (gcontroller.getRightTriggerAxis() > 0.75)
        {
            feederMotor.set(ShooterConstants.FEEDER_SPEED);
        }
        else if (gcontroller.getLeftTriggerAxis() > 0.75)
        {
            feederMotor.set(-ShooterConstants.FEEDER_SPEED);
        }
        else
        {
            feederMotor.set(0);
        }

        // Shooter Backwards
        if (gcontroller.getXButton()) 
        {
            shooterLeft.set(-ShooterConstants.SHOOTER_REVERSE_SPEED);
            shooterRight.set(ShooterConstants.SHOOTER_REVERSE_SPEED);
        }
        // Speaker Button
        else if (gcontroller.getYButton())
        {
            shooterLeft.set(ShooterConstants.SPEAKER_SPEED);
            shooterRight.set(-ShooterConstants.SPEAKER_SPEED);
        }
        // AMP Button
        else if (gcontroller.getAButton())
        {
            shooterLeft.set(ShooterConstants.AMP_SPEED);
            shooterRight.set(-ShooterConstants.AMP_SPEED);
        }
        else 
        {
            shooterLeft.set(0);
            shooterRight.set(0);
        }
    }
}
