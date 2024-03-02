// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ColourSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimitSwitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  // The driver's controller
  public static CommandXboxController driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static XboxController operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  public final DriveSubsystem robotDrive = new DriveSubsystem();
  public final IntakeSubsystem robotIntake = new IntakeSubsystem(operatorController);
  public final ShooterSubsystem robotShooter = new ShooterSubsystem(operatorController);
  public final ElevatorSubsystem robotElevator = new ElevatorSubsystem(operatorController);
  public final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public final ColourSensorSubsystem colourSensorSubsystem = new ColourSensorSubsystem();
  public final LimitSwitchSubsystem limitSwitchSubsystem = new LimitSwitchSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              robotDrive.drive(
                  Math.pow(MathUtil.applyDeadband(-driverController.getLeftY(), ModuleConstants.DEADBAND), 3),
                  Math.pow(MathUtil.applyDeadband(-driverController.getLeftX(), ModuleConstants.DEADBAND), 3),
                  Math.pow(MathUtil.applyDeadband(driverController.getRawAxis(4), ModuleConstants.DEADBAND), 3),
                  true);
            },
            robotDrive));

    /* Configure default commands */
    cameraSubsystem.setDefaultCommand(
        /* Prints estimated pose to SmartDashboard */
        new RunCommand(
            () -> {
              Pose3d estimatedPose = cameraSubsystem.getLastEstimatedRobotPose(false);
              SmartDashboard.putNumberArray("Camera Estimated Position",
                  new double[] { estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getZ() });
              SmartDashboard.putNumberArray("Camera Estimated Rotation",
                  new double[] { Math.toDegrees(estimatedPose.getRotation().getX()),
                      Math.toDegrees(estimatedPose.getRotation().getY()),
                      Math.toDegrees(estimatedPose.getRotation().getZ()) });
            },
            cameraSubsystem));
    /* Prints current status of limit switches to SmartDashboard */
    // limitSwitchSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       for (int i = 0; i < Constants.ModuleConstants.LIMIT_SWITCHES.length; i++) {
    //         SmartDashboard.putBoolean("Limit Switch #" + i, limitSwitchSubsystem.isClosed(i));
    //       }
    //     },
    //     limitSwitchSubsystem
    //   )
    // );

    robotElevator.setDefaultCommand(
      new RunCommand
      (
        () -> {
          robotElevator.elevatorControl();
        }, robotElevator
      )
    );

    colourSensorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              SmartDashboard.putNumber("Colour Sensor Proximity", colourSensorSubsystem.getProximity());
              SmartDashboard.putString("Colour Sensor Detected Colour",
                  colourSensorSubsystem.getDetectedColour().toHexString());
            },
            colourSensorSubsystem));
    robotIntake.setDefaultCommand(
        new RunCommand(
            () -> {
              robotIntake.intake();
            }, robotIntake));

    robotShooter.setDefaultCommand(
        new RunCommand(
            () -> {
              robotShooter.buttonShoot();
            }, robotShooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // EventLoop event = new EventLoop();
    // event.bind(
    //   () -> {
    //     System.out.println("Hello");
    //   }
    // );
    // operatorController.button(2, event);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   //// Create config for trajectory
  //   TrajectoryConfig config =
  //      new TrajectoryConfig(
  //              AutoConstants.MAX_SPEED_METERS_PER_SECOND,
  //              AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
  //          // Add kinematics to ensure max speed is actually obeyed
  //          .setKinematics(DriveConstants.DRIVE_KINEMATICS);
  //   // An example trajectory to follow.  All units in meters.
  //   Trajectory exampleTrajectory =
  //      TrajectoryGenerator.generateTrajectory(
  //          // Start at the origin facing the +X direction
  //          new Pose2d(0, 0, new Rotation2d(0)),
  //          // Pass through these two interior waypoints, making an 's' curve path
  //          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  //          // End 3 meters straight ahead of where we started, facing forward
  //          new Pose2d(3, 0, new Rotation2d(0)),
  //          config);
    
  //   SwerveControllerCommand swerveControllerCommand = getSwerveControllerCommand(exampleTrajectory);
    
  //   // Reset odometry to the starting pose of the trajectory.
  //   robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
  // }
  

  // private SwerveControllerCommand getSwerveControllerCommand(Trajectory exampleTrajectory) {
  //   var thetaController =
  //       new ProfiledPIDController(
  //           AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand =
  //       new SwerveControllerCommand(
  //               exampleTrajectory,
  //           robotDrive::getPose, // Functional interface to feed supplier
  //           DriveConstants.DRIVE_KINEMATICS,

  //           // Position controllers
  //           new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
  //           new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
  //           thetaController,
  //           robotDrive::setModuleStates,
  //               robotDrive);
  //   return swerveControllerCommand;

  // }
} 
