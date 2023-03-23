// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   *
   * <p>Currently initializes button bindings & dashboard.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.roboInit();

    CameraServer.startAutomaticCapture();
  }

  /**
   * Poll for events every 20ms.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * Initialize disabled mode
   *
   * <p>Disables elevator
   */
  @Override
  public void disabledInit() {
    m_robotContainer.disableElevatorPID();
  }

  /**
   * Poll events in disabled mode
   *
   * <p>Should always be none?
   */
  @Override
  public void disabledPeriodic() {}

  /** Initialize autonomous mode */
  @Override
  public void autonomousInit() {
    m_robotContainer.autoInit();
  }

  /** Poll events (tilt) in autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autoPeriodic();
  }

  /**
   * Enables tele/remote operation of the Robot.
   *
   * <p>Cancels any autonomous commands.
   */
  @Override
  public void teleopInit() {
    // Halt any running commands
    // TODO: Why not use CommandScheduler.getInstance().cancelAll() like in testInit()?
    if (m_robotContainer.m_autonomousCommand != null) {
      m_robotContainer.m_autonomousCommand.cancel();
    }

    m_robotContainer.teleOperatedInit();
  }

  /** Polls for inputs periodically */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleoperatedPeriodic();
  }

  /** Runs test mode, halting all commands */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Poll Test Mode */
  @Override
  public void testPeriodic() {}

  /** Initialize Simulation */
  @Override
  public void simulationInit() {}

  /** Poll Simulation Mode */
  @Override
  public void simulationPeriodic() {}
}
