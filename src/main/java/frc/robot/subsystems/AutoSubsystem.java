package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSubsystem
{
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private Timer timer;

    public AutoSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;
        timer = new Timer();
    }   
    
    public SequentialCommandGroup startAutos()
    {
        SequentialCommandGroup group = new SequentialCommandGroup();

        group.addCommands
        (
            shooter.SpeakerShooter(),
            new WaitCommand(1),
            shooter.ShooterStop()
            
        );  
        
        return group;
    }
}
