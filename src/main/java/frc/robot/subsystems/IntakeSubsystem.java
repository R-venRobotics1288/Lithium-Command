package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase
{
    private CANSparkMax intakingMotor;
    private CANSparkMax positioningMotor;
    private CANSparkMax feederMotor;

    private XboxController gcontroller;
    private ColourSensorSubsystem colourSensorSubsystem;

    private Color detectedColor;

    public IntakeSubsystem(XboxController controller) 
    {
        intakingMotor = new CANSparkMax(DriveConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        positioningMotor = new CANSparkMax(DriveConstants.POSITION_MOTOR_PORT, MotorType.kBrushless);
        feederMotor = new CANSparkMax(DriveConstants.FEEDER_MOTOR_PORT, MotorType.kBrushless);
        
        intakingMotor = null;
        positioningMotor = null;
        feederMotor = null;

        colourSensorSubsystem = new ColourSensorSubsystem();

        gcontroller = controller;

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
        detectedColor = colourSensorSubsystem.getDetectedColour();
    }
    
    public void intake()
    {
        // detectedColor.red > detectedColor.green 
        if (gcontroller.getRightBumper())
        {
            intakingMotor.set(IntakeConstants.INTAKING_SPEED);
            positioningMotor.set(IntakeConstants.POSITIONING_SPEED);
            feederMotor.set(IntakeConstants.FEEDER_SPEED);
        }
        else if (gcontroller.getLeftBumper())
        {
            intakingMotor.set(-IntakeConstants.INTAKING_SPEED);
            positioningMotor.set(-IntakeConstants.POSITIONING_SPEED);
            feederMotor.set(-IntakeConstants.FEEDER_SPEED);
        }
        else 
        {
            intakingMotor.set(0);
            positioningMotor.set(0);
            feederMotor.set(0);
        }
    }
    
}
