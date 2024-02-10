package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class ShooterSubsytem extends SubsystemBase {
    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax feederMotor;

    private XboxController gcontroller;
    private CameraSubsystem cameraSubsystem;

    Pose3d robotPositonToApril;

    public ShooterSubsytem(XboxController controller) {
        // feederMotor = new CANSparkMax(DriveConstants.FEEDER_MOTOR_PORT,
        // MotorType.kBrushless);
        // shooterLeft = new CANSparkMax(DriveConstants.LEFT_SHOOTER_MOTOR_PORT,
        // MotorType.kBrushed);
        // shooterRight = new CANSparkMax(DriveConstants.RIGHT_SHOOTER_MOTOR_PORT,
        // MotorType.kBrushed);
        gcontroller = controller;

        robotPositonToApril = new Pose3d();
        cameraSubsystem = new CameraSubsystem();

        // shooterLeft.setSmartCurrentLimit(80);
        // shooterRight.setSmartCurrentLimit(80);
        // feederMotor.setSmartCurrentLimit(80);

        // shooterLeft.burnFlash();
        // shooterRight.burnFlash();
        // feederMotor.burnFlash();
    }

    @Override
    public void periodic() {
        checkAprilTag();
    }

    private Command checkAprilTag() {
        return this.runOnce(() -> robotPositonToApril = cameraSubsystem.getLastEstimatedRobotPose(false));
    }

    public Command cameraAutoAlign() {
        return this.runOnce(() -> {
                if (gcontroller.getRawButton(1)) {
                    System.out.println("Hi");
                }
        });

    }

    public Command buttonShoot() {
        return this.run(() -> {
            // Speaker button - 8, AMP Button - 7
            if (gcontroller.getRawButton(OIConstants.SPEAKER_BUTTON_PORT)
                    || gcontroller.getRawButton(OIConstants.AMP_BUTTON_PORT)) 
            {
                // feederMotor.set(0.3);
                System.out.println("Shoot!");
            } 
            else 
            {
                // shooterLeft.set(0);
                // shooterRight.set(0);
                // feederMotor.set(0);
            }
        });

    }
}
