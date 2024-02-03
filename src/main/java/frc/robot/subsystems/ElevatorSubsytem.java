package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

public class ElevatorSubsytem extends SubsystemBase
{
    private CANSparkMax leftElevatorMotor;
    private CANSparkMax rightElevatorMotor;

    private XboxController gController;

    public ElevatorSubsytem(XboxController controller)
    {
        leftElevatorMotor = new CANSparkMax(16,MotorType.kBrushless);
        rightElevatorMotor = new CANSparkMax(17, MotorType.kBrushless);

        gController = controller;

        leftElevatorMotor.setSmartCurrentLimit(80);
        rightElevatorMotor.setSmartCurrentLimit(80);
        leftElevatorMotor.burnFlash();
        rightElevatorMotor.burnFlash();
    }

    @Override
    public void periodic()
    {
        
    }

    public void elevatorControl()
    {
        if(gController.getRawButton(OIConstants.ELEVATOR_UP_BUTTON_PORT))
        {
            leftElevatorMotor.set(0.35);
            rightElevatorMotor.set(0.35);
        }
        else if(gController.getRawButton(OIConstants.ELEVATOR_DOWN_BUTTON_PORT))
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
