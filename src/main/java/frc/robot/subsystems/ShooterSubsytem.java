package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsytem extends SubsystemBase {
    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax feederMotor;

    private final XboxController gcontroller = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    Pose3d robotPositonToApril;

    public ShooterSubsytem(CommandXboxController controller, CANSparkMax feederMotor) {
        this.feederMotor = feederMotor;
        shooterLeft = new CANSparkMax(DriveConstants.LEFT_SHOOTER_MOTOR_PORT,
        MotorType.kBrushless);
        shooterRight = new CANSparkMax(DriveConstants.RIGHT_SHOOTER_MOTOR_PORT,
        MotorType.kBrushless);
        // gcontroller = controller;
        
        // robotPositonToApril = new Pose3d();

        shooterLeft.setSmartCurrentLimit(80);
        shooterRight.setSmartCurrentLimit(80);
        feederMotor.setSmartCurrentLimit(80);
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

    public Command FeederMotorReverse() 
    {
        return this.run(() -> {
            feederMotor.set(ShooterConstants.FEEDER_SPEED);
        });
    }

    public Command FeederMotorForward()
    {
        return this.run(() -> {
            feederMotor.set(-ShooterConstants.FEEDER_SPEED);
        });
    }   

    public Command FeederStop()
    {
        return this.runOnce(() -> {
            feederMotor.set(0);
        });
    }

    public Command ShooterReverse()
    {
        return this.run(() -> {
            shooterLeft.set(-ShooterConstants.SHOOTER_REVERSE_SPEED);
            shooterRight.set(ShooterConstants.SHOOTER_REVERSE_SPEED);
        });
    }

    public Command SpeakerShooter()
    {
        return this.run(() -> {
            shooterLeft.set(ShooterConstants.SPEAKER_SPEED);
            shooterRight.set(-ShooterConstants.SPEAKER_SPEED);
        });
    }

    public Command AMPShooter()
    {
        return this.run(() -> {
            shooterLeft.set(ShooterConstants.AMP_SPEED);
            shooterRight.set(-ShooterConstants.AMP_SPEED);
        });
    }

    public Command ShooterStop()
    {
        return this.runOnce(() -> {
            shooterLeft.set(0);
            shooterRight.set(0);
        });
    }

    // TODO: event driven button input
    public void buttonShoot() {
        // Feeder Motor
        if (gcontroller.getRightTriggerAxis() > 0.75)
        {
        }
        // Feeder Revserse
        else if (gcontroller.getLeftTriggerAxis() > 0.75)
        {
        }
        else
        {
        }

        // Shooter Backwards
        if (gcontroller.getXButton()) 
        {
          
        }
        // Speaker Button
        else if (gcontroller.getYButton())
        {
           
        }
        // AMP Button
        else if (gcontroller.getAButton())
        {
            
        }
        else 
        {
           
        }
    }
}
