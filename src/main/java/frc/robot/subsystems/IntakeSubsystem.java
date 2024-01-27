package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{
    private CANSparkMax intakingMotor;
    private CANSparkMax positioningMotor;

    private XboxController gcontroller;

    public IntakeSubsystem(XboxController controller) {
        intakingMotor = new CANSparkMax(11, MotorType.kBrushless);
        positioningMotor = new CANSparkMax(12, MotorType.kBrushless);
        
        gcontroller = controller;

        intakingMotor.setSmartCurrentLimit(80);
        positioningMotor.setSmartCurrentLimit(80);
        intakingMotor.burnFlash();
        positioningMotor.burnFlash();
    }

    @Override
    public void periodic()
    {
        
    }

    public void intake()
    {
        if (gcontroller.getRawButton(7))
        {
            intakingMotor.set(0.8);
            positioningMotor.set(-0.6);
        }
        else 
        {
            intakingMotor.set(0);
            positioningMotor.set(0);
        }
    }
    
}