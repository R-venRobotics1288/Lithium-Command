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
        System.out.println(mode);
        System.out.println(alliance);
        SequentialCommandGroup group = new SequentialCommandGroup();


        switch (alliance) 
        {
            case RED:
                switch (mode) 
                {
                    case LEFT:
                        group.addCommands
                        (
                            shooter.SpeakerShooter(),
                            new WaitCommand(0.5),
                            shooter.FeederMotorForward(),
                            new WaitCommand(1),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            new WaitCommand(0.5),
                            intake.IntakeForward(),
                            drive.AutoDrive(-0.3, 0, 0, true, 0.25)
                        );
                        break;
                    case CENTER:

                        final double DRIVE_SPEED = 0.15;
                        final double DRIVE_RUNTIME = 1.5;

                        group.addCommands
                        (
                            shooter.SpeakerShooter(),
                            new WaitCommand(1.5),
                            shooter.FeederMotorReverse(),
                            new WaitCommand(0.3),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            new WaitCommand(0.5),
                            intake.IntakeForward(),
                            drive.AutoDrive(0, -DRIVE_SPEED, 0, true, DRIVE_RUNTIME),
                           // new WaitCommand(1.2),
                            intake.IntakeStop()
                            // drive.AutoDrive(0, DRIVE_SPEED, 0, true, DRIVE_RUNTIME),
                            // shooter.SpeakerShooter(),
                            // new WaitCommand(0.5),
                            // shooter.FeederMotorReverse(),
                            // new WaitCommand(1),
                            // shooter.ShooterStop(),
                            // shooter.FeederStop(),
                            // drive.AutoDrive(0, -DRIVE_SPEED, 0, true, DRIVE_RUNTIME)
                        );
                        break;
                    case RIGHT:
                        group.addCommands
                        (
                            shooter.SpeakerShooter(),
                            new WaitCommand(0.5),
                            shooter.FeederMotorForward(),
                            new WaitCommand(1),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            new WaitCommand(0.5),
                            intake.IntakeForward(),
                            drive.AutoDrive(0.3, 0, 0, true, 0.25)
                        );
                        break;
                    default:
                        System.out.println("Red default");
                        break;
                }
                break;
            case BLUE:
                switch (mode) 
                {
                    case LEFT:   
                        group.addCommands
                        (
                            drive.AutoDrive(0, 0, 0.3, true, 0.5)
                        );
                        break;
                    case CENTER:
                        group.addCommands
                        (
                            shooter.SpeakerShooter(),
                            new WaitCommand(0.5),
                            shooter.FeederMotorReverse(),
                            new WaitCommand(1),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            new WaitCommand(0.5),
                            intake.IntakeForward(),
                            drive.AutoDrive(0, -0.5, 0, true, 0.25),
                            new WaitCommand(1.2),
                            intake.IntakeStop(),
                            drive.AutoDrive(0, 0.5, 0, true, 0.25),
                            shooter.SpeakerShooter(),
                            new WaitCommand(0.5),
                            shooter.FeederMotorReverse(),
                            new WaitCommand(1),
                            shooter.ShooterStop(),
                            shooter.FeederStop(),
                            drive.AutoDrive(0, -0.5, 0, true, 0.25)
                        );
                        break;
                    case RIGHT:
                      group.addCommands
                        (
                            drive.AutoDrive(0, 0, -0.3, true, 0.5)
                        );
                        break;
                    default:
                        System.out.println("Blue default");
                        break;
                }
            default:
                System.out.println("No Alliance default");
                break;
        }
     
        
        return group;
    }
}
