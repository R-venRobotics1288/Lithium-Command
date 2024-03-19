package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSubsystem
{
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DriveSubsystem drive;

    public AutoSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem drive) 
    {
        this.shooter = shooter;
        this.intake = intake;
        this.drive = drive;
    }   
    
    public SequentialCommandGroup startAutos(String mode)
    {
        SequentialCommandGroup group = new SequentialCommandGroup();

        switch (mode) 
        {
            case "center":
                group.addCommands
                (
                    shooter.SpeakerShooter(),
                    shooter.FeederMotorForward(),
                    new WaitCommand(1),
                    shooter.ShooterStop(),
                    shooter.FeederStop(),
                    new WaitCommand(2),
                    drive.AutoDrive(0, -0.5, 0, true, 5)
                );
                break;
        
            default:
                break;
        }
     
        
        return group;
    }
}
