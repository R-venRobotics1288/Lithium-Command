package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AutoSubsystem extends SubsystemBase
{
    private ShooterSubsystem shooter;
    private Robot robot;

    public AutoSubsystem() {
        shooter = new ShooterSubsystem(robot.robotContainer.feederMotor);
        robot = new Robot();
    }

    @Override
    public void periodic()
    {}

    public Command startAutos()
    {
        return this.run(() -> {
            shooter.SpeakerShooter();
            shooter.ShooterStop();
        });
        
    }
}
