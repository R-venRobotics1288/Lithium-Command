package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase
{
    private XboxController controller;
    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax feederMotor;

    public ShooterSubsytem(XboxController controller)
    {
       shooterLeft = new CANSparkMax(14, MotorType.kBrushed);
       shooterRight = new CANSparkMax(15, MotorType.kBrushed);
       feederMotor = new CANSparkMax(13, MotorType.kBrushless);
        this.controller = controller;

        shooterLeft.setSmartCurrentLimit(80);
        shooterRight.setSmartCurrentLimit(80);
        feederMotor.setSmartCurrentLimit(80);
        shooterLeft.burnFlash();
        shooterRight.burnFlash();
        feederMotor.burnFlash();
    }
    
    @Override
    public void periodic()
    {

    }

    public void buttonShoot()
    {
        // Speaker button - 8, AMP Button - 7
        if(controller.getRawButton(8) || controller.getRawButton(7))
        {
            feederMotor.set(0.3);
        }
        else 
        {
            shooterLeft.set(0);
            shooterRight.set(0);
            feederMotor.set(0);
        }
    }
}
