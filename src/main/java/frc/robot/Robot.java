// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Alliance;
import frc.robot.Constants.AutoMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private Command alignmentCommand;
  private Command shooterCommand;

  public RobotContainer robotContainer = new RobotContainer();
  private SendableChooser modeChooser;
  private SendableChooser allianceChooser;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

     
    modeChooser = new SendableChooser<>();
    allianceChooser = new SendableChooser<>();
    CameraServer.startAutomaticCapture();

    allianceChooser.addOption("Red", Alliance.RED);    
    allianceChooser.addOption("Blue", Alliance.BLUE);

    modeChooser.addOption("Left", AutoMode.LEFT);
    modeChooser.addOption("Center", AutoMode.CENTER);
    modeChooser.addOption("Right", AutoMode.RIGHT);

    // robotContainer.robotDrive.zeroHeading(); // probably not needed any more -- test for certainty

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // CommandScheduler.getInstance().schedule(alignmentCommand, shooterCommand);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putData("Alliance", allianceChooser);
    SmartDashboard.putData("Position", modeChooser);

    SmartDashboard.putNumber("Front Left Encoder", Math.toDegrees(robotContainer.robotDrive.getFrontLeft()));
    SmartDashboard.putNumber("Front Right Encoder", Math.toDegrees(robotContainer.robotDrive.getFrontRight()));
    SmartDashboard.putNumber("Rear Left Encoder", Math.toDegrees(robotContainer.robotDrive.getRearLeft()));
    SmartDashboard.putNumber("Rear Right Encoder", Math.toDegrees(robotContainer.robotDrive.getRearRight()));
    SmartDashboard.putNumber("Left Joystick X", robotContainer.driverController.getLeftX());
    SmartDashboard.putNumber("Left Joystick Y", robotContainer.driverController.getLeftY());
    SmartDashboard.putNumber("Right Joystick X", robotContainer.driverController.getRawAxis(4));
    SmartDashboard.putNumberArray("Desired State", robotContainer.robotDrive.getDesiredState());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override 
  public void autonomousInit() 
  {
    System.out.println(modeChooser.getSelected());
    System.out.println(allianceChooser.getSelected());
    robotContainer.auto((AutoMode)modeChooser.getSelected(), (Alliance)allianceChooser.getSelected()).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
        // CommandScheduler.getInstance().run();

  }
}
