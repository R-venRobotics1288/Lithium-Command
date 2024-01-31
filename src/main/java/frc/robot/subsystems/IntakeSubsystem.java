package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase
{
    private CANSparkMax intakingMotor;
    private CANSparkMax positioningMotor;
    private CANSparkMax feederMotor;

    private XboxController gcontroller;

    private RobotContainer robotContainer;
    private Color detectedColor;

    public IntakeSubsystem(XboxController controller) 
    {
        intakingMotor = new CANSparkMax(11, MotorType.kBrushless);
        positioningMotor = new CANSparkMax(12, MotorType.kBrushless);
        feederMotor = new CANSparkMax(13, MotorType.kBrushless);
        
        gcontroller = controller;

        robotContainer = new RobotContainer();

        intakingMotor.setSmartCurrentLimit(80);
        positioningMotor.setSmartCurrentLimit(80);
        feederMotor.setSmartCurrentLimit(80);
        intakingMotor.burnFlash();
        positioningMotor.burnFlash();
        feederMotor.burnFlash();
    }

    @Override
    public void periodic()
    {
      grabColorDetector();
    }

    public Command grabColorDetector()
    {
        return this.runOnce(
            () -> 
                detectedColor = robotContainer.colourSensorSubsystem.getDetectedColour()
            );
    }

    public void intake()
    {
        if (detectedColor.equals(Color.kOrange))
        {
            System.out.println("Orange");
        }
        if (gcontroller.getRawButton(5))
        {
            intakingMotor.set(0.8);
            positioningMotor.set(-0.6);
            feederMotor.set(0.4);
        }
        else 
        {
            intakingMotor.set(0);
            positioningMotor.set(0);
            feederMotor.set(0);
        }
    }
    
}
