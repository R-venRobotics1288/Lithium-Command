package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

    private CommandXboxController gcontroller;
    private ColourSensorSubsystem colourSensorSubsystem;

    private Color detectedColor;

    public IntakeSubsystem(CommandXboxController controller, CANSparkMax feederMotor) 
    {
        this.feederMotor = feederMotor;
        intakingMotor = new CANSparkMax(DriveConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        positioningMotor = new CANSparkMax(DriveConstants.POSITION_MOTOR_PORT, MotorType.kBrushless);

        colourSensorSubsystem = new ColourSensorSubsystem();
        gcontroller = controller;
    }

    @Override
    public void periodic()
    {
      
      
        detectedColor = colourSensorSubsystem.getDetectedColour();
    }
    
    public Command IntakeForward() {
        return this.run(() -> {
            intakingMotor.set(IntakeConstants.INTAKING_SPEED);
            positioningMotor.set(IntakeConstants.POSITIONING_SPEED);
            feederMotor.set(IntakeConstants.FEEDER_SPEED);
        });
    };

    public Command IntakeReverse()
    {
        return this.run(() -> {
            intakingMotor.set(-IntakeConstants.INTAKING_SPEED);
            positioningMotor.set(-IntakeConstants.POSITIONING_SPEED);
            feederMotor.set(-IntakeConstants.FEEDER_SPEED);
        });
    }

    public Command IntakeStop()
    {
        return this.runOnce(() -> {
            intakingMotor.set(0);
            positioningMotor.set(0);
            feederMotor.set(0);
        });
    }

    public void intake()
    {
     
        // detectedColor.red > detectedColor.green 
       
    }
    
}
