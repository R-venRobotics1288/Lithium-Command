// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ColourSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.subsystems.LimitSwitchSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public final ColourSensorSubsystem colourSensorSubsystem = new ColourSensorSubsystem();
  public final LimitSwitchSubsystem limitSwitchSubsystem = new LimitSwitchSubsystem();
  public final ShooterSubsystem shooterSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // The driver's controller
  public final CommandXboxController driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  public final XboxController operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialise subsystems dependent on controllers.
    shooterSubsystem = new ShooterSubsystem(operatorController);
    intakeSubsystem = new IntakeSubsystem(operatorController);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              driveSubsystem.drive(
                  Math.pow(MathUtil.applyDeadband(-driverController.getLeftY(), ModuleConstants.DEADBAND), 3),
                  Math.pow(MathUtil.applyDeadband(-driverController.getLeftX(), ModuleConstants.DEADBAND), 3),
                  Math.pow(MathUtil.applyDeadband(driverController.getRawAxis(4), ModuleConstants.DEADBAND), 3),
                  true);
            },
            driveSubsystem));

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
    /* Prints Colour Sensor information to SmartDashboard */
    colourSensorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              SmartDashboard.putNumber("Colour Sensor Proximity", colourSensorSubsystem.getProximity());
              SmartDashboard.putString("Colour Sensor Detected Colour",
                  colourSensorSubsystem.getDetectedColour().toHexString());
            },
            colourSensorSubsystem));
    intakeSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              intakeSubsystem.intake();
            }, intakeSubsystem));

    intakeSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              intakeSubsystem.intake();;
            }, intakeSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // HOW TO RUN COMMAND ON BUTTON PRESS -> driverController.x().onTrue(new RunCommand(() -> {}, robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("RavenCompetition");
  }
}