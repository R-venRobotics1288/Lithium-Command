package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoMode;
import frc.robot.Constants.Alliance;

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
    
    public SequentialCommandGroup startAutos(AutoMode mode, Alliance alliance)
    {
        SequentialCommandGroup group = new SequentialCommandGroup();

        switch (alliance) 
        {
            case RED:
                switch (mode) 
                {
                    case LEFT:
                        break;
                    case CENTER:
                        group.addCommands
                        (
                            shooter.SpeakerShooter(),
                            new WaitCommand(0.5),
                            shooter.FeederMotorForward(),
                            new WaitCommand(1),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            new WaitCommand(2),
                            drive.AutoDrive(0, -0.5, 0, true, 5)
                        );
                        break;
                    case RIGHT:
                        break;
                    default:
                        break;
                }
                break;
            case BLUE:
                switch (mode) 
                {
                    case LEFT:   
                        break;
                    case CENTER:
                        group.addCommands
                        (
                            shooter.SpeakerShooter(),
                            new WaitCommand(0.5),
                            shooter.FeederMotorForward(),
                            new WaitCommand(1),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            new WaitCommand(2),
                            drive.AutoDrive(0, -0.5, 0, true, 5)
                        );
                        break;
                    case RIGHT:
                        break;
                    default:
                        break;
                }
            default:
                break;
        }
     
        
        return group;
    }
}
