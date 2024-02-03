package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;

public class ElevatorSubsytem extends SubsystemBase
{
    private CANSparkMax leftElevatorMotor;
    private CANSparkMax rightElevatorMotor;
    private RelativeEncoder rightElevatorEncoder;
    private RelativeEncoder leftElevatorEncoder;

    private XboxController gController;

    public ElevatorSubsytem(XboxController controller)
    {
        leftElevatorMotor = new CANSparkMax(DriveConstants.LEFT_ELEVATOR_MOTOR_PORT,MotorType.kBrushless);
        rightElevatorMotor = new CANSparkMax(DriveConstants.RIGHT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);

        
        gController = controller;

        leftElevatorMotor.setSmartCurrentLimit(80);
        rightElevatorMotor.setSmartCurrentLimit(80);
        leftElevatorMotor.burnFlash();
        rightElevatorMotor.burnFlash();

        rightElevatorEncoder = rightElevatorMotor.getEncoder();
        leftElevatorEncoder = leftElevatorMotor.getEncoder();
    }

    @Override
    public void periodic()
    {
        
    }

    public void elevatorControl()
    {
        if((leftElevatorEncoder.getPosition() < IntakeConstants.TOP_ELEVATOR_LIMIT || rightElevatorEncoder.getPosition() < IntakeConstants.BOT_ELEVATOR_LIMIT) && gController.getRawButton(OIConstants.ELEVATOR_UP_BUTTON_PORT))
        {
            leftElevatorMotor.set(0.35);
            rightElevatorMotor.set(0.35);
        }
        else if((leftElevatorEncoder.getPosition() > IntakeConstants.BOT_ELEVATOR_LIMIT || rightElevatorEncoder.getPosition() > IntakeConstants.BOT_ELEVATOR_LIMIT) && gController.getRawButton(OIConstants.ELEVATOR_DOWN_BUTTON_PORT))
        {
            leftElevatorMotor.set(-0.35);
            rightElevatorMotor.set(-0.35);
        }
        else
        {
            leftElevatorMotor.set(0);
            rightElevatorMotor.set(0);
        }
    }
}
