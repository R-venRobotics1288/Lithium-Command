package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;

public class ElevatorSubsystem extends SubsystemBase
{
    private CANSparkMax leftElevatorMotor;
    private CANSparkMax rightElevatorMotor;
    private RelativeEncoder rightElevatorEncoder;
    private RelativeEncoder leftElevatorEncoder;

    private XboxController gController;

    public ElevatorSubsystem(XboxController controller)
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
        if((leftElevatorEncoder.getPosition() < ElevatorConstants.TOP_ELEVATOR_LIMIT || rightElevatorEncoder.getPosition() < ElevatorConstants.BOT_ELEVATOR_LIMIT) && gController.getLeftY() > ModuleConstants.DEADBAND)
        {
            leftElevatorMotor.set(ElevatorConstants.LEFT_MOTOR_SPEED);
            rightElevatorMotor.set(ElevatorConstants.RIGHT_MOTOR_SPEED);
        }
        else if((leftElevatorEncoder.getPosition() > ElevatorConstants.BOT_ELEVATOR_LIMIT || rightElevatorEncoder.getPosition() > ElevatorConstants.BOT_ELEVATOR_LIMIT) && gController.getLeftY() < -ModuleConstants.DEADBAND)
        {
            leftElevatorMotor.set(-ElevatorConstants.LEFT_MOTOR_SPEED);
            rightElevatorMotor.set(-ElevatorConstants.RIGHT_MOTOR_SPEED);
        }
        else
        {
            leftElevatorMotor.set(0);
            rightElevatorMotor.set(0);
        }
    }
}
