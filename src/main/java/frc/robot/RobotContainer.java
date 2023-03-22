// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoBalanceB;
import frc.robot.commands.AutoBalanceF;
import frc.robot.commands.ConeRelease;
import frc.robot.commands.CubeRelease;
import frc.robot.commands.ElevatorRaiseMid;
import frc.robot.commands.ElevatorRaiseTop;
import frc.robot.commands.IntakeRelease;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.LowerELevator;
import frc.robot.commands.LowerIntake;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.autoBalance;
import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks & Xbox Controllers
  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(opJoy, 1);
  public final JoystickButton bButton = new JoystickButton(opJoy, 2);
  public final JoystickButton xButton = new JoystickButton(opJoy, 3);
  public final JoystickButton startButton = new JoystickButton(opJoy, 8);

  public final POVButton upPOV = new POVButton(opJoy, 90);
  public final POVButton downPOV = new POVButton(opJoy, 0);

  public final ElevatorPID m_elevatorPID = new ElevatorPID();
  public final DriveBase m_DriveBase = new DriveBase();
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final autoBalance m_autoBalance = new autoBalance();

  public Trajectory trajectory;
  public Command m_autonomousCommand;
  public SendableChooser<String> autoChooser = new SendableChooser<String>();

  /*
   * Manages the 3 states of the roller
   * 1. Cone
   * 2. Cube
   * 3. Neither
   */
  public enum RollerState {
    CONE,
    CUBE,
    NEITHER
  }

  // Last held object by Roller
  RollerState lastHeld;

  /*
   * Normalized magnitude for a given axis of xbox Controller.
   *
   * <p>Axises:
   * 1. LX Axis
   * 2. LY Axis
   * 3. L Trigger
   * 4. R Trigger
   * 5. RX Axis
   * 6. RY Axis
   * </p>
   *
   * @param axis joystick Axis
   * @return range between 1-0.
   */
  public double getDriveJoy(int axis) {
    return normalizeJoy(driveJoy.getRawAxis(axis));
  }

  /*
   * Gets the absolute value of the raw input (range -1 -> 1 becomes 0 -> 1).
   * Then ignores any magnitudes less than 0.1 (dead-zone).
   *
   * @param raw joystick value
   * @return axis magnitude
   */
  private double normalizeJoy(double raw) {
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }

  /*
   * Magnitude of the x-axis of right joystick.
   * Used for Rotation
   *
   * @return magnitude
   */
  public double getDriveJoyXR() {
    return getDriveJoy(4) / 2.5;
  }

  /*
   * Magnitude of the y-axis of left joystick.
   * Used for Forward
   *
   * @return magnitude
   */
  public double getDriveJoyYL() {
    return getDriveJoy(1);
  }

  /*
   * Null constructor
   * Simply builds class as a container and runs the individual *Init() functions.
   */
  public RobotContainer() {}

  /*
   * Initialize Smart-dashboard
   */
  public void roboInit() {
    autoChooser.addOption("CubeLow+AutoDock", "PreloadPath1B");
    autoChooser.addOption("MobilityPathBackwards", "DockPath");
    autoChooser.addOption("MobilityPathForwards", "DockPath2");
    autoChooser.addOption("Dock Backwards", "AutoBalanceDockBack");
    autoChooser.addOption("Dock Forwards", "AutoBalanceDockForward");
    autoChooser.addOption("ConeHigh+Dock", "ConeHigh+Dock");
    autoChooser.addOption("ConeMid", "ConeMid+Dock");
    autoChooser.addOption("ConeMid+Dock", "ConeMid");
    autoChooser.addOption("CubeHigh + Dock", "CubeHigh+Dock");
    autoChooser.addOption("Cone+Mobility+Dock", "Cone+Mobility+Dock");
    autoChooser.addOption("Cube+Mobility+Dock", "Cube+Mobility+Dock");
    autoChooser.addOption("CubeHigh+Mobility", "CubeHigh+Mobility");
    autoChooser.addOption("ConeHigh+Mobility", "ConeHigh+Mobility");
    autoChooser.addOption("CubeHigh+Engage", "Cube+Engage");
    autoChooser.addOption("CubeLow+Mobility", "CubeLow+Mobility");

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  /*
   * Initializes autonomous control
   */
  public void autoInit() {
    m_DriveBase.m_gyro.reset();
    m_DriveBase.resetEncoders();
    m_elevatorPID.encoderR.setPosition(0);

    if (autoChooser.getSelected() != null) {
      m_autonomousCommand = getAutonomousCommand(autoChooser.getSelected());
      m_autonomousCommand.schedule();
    }
  }

  /*
   * Adds the tilt to smart-dashboard
   */
  public void autoPeriodic() {
    m_autoBalance.getTilt();
    SmartDashboard.putNumber("Tilt", m_autoBalance.getTilt());
  }

  /*
   * Resets device and allows for remote control
   */
  public void teleOperatedInit() {
    m_DriveBase.resetEncoders();
    lastHeld = RollerState.NEITHER;
  }

  public void teleoperatedPeriodic() {
    SmartDashboard.putNumber("Encoder Right", m_elevatorPID.encoderR.getPosition());

    // Driving Junk
    m_DriveBase.m_drive.curvatureDrive(-getDriveJoyYL(), -getDriveJoyXR(), true);

    // Elevator Set-points - Magic Numbers?
    final double setpoint1 = -20;
    final double setpoint2 = -54;
    final double setpoint3 = -62;

    // Elevator Buttons
    if (opJoy.getAButton())
      m_elevatorPID.setGoal(0);
    else if (opJoy.getBButton())
      m_elevatorPID.setGoal(setpoint1);
    else if (opJoy.getYButton())
      m_elevatorPID.setGoal(setpoint2);
    else if (opJoy.getXButton())
      m_elevatorPID.setGoal(setpoint3);
    else if (opJoy.getAButton())

    enableElevatorPID();

    // Roller Buttons
    if (opJoy.getRightBumper()) {
      m_intakeSub.intakeRoll.set(1);
      lastHeld = RollerState.CONE;
    }else if (opJoy.getLeftBumper()) {
      m_intakeSub.intakeRoll.set(-0.8);
      lastHeld = RollerState.CUBE;
    } else if (lastHeld == RollerState.CONE) {
      m_intakeSub.intakeRoll.set(0.2);
    } else if (lastHeld == RollerState.CUBE) {
      m_intakeSub.intakeRoll.set(-0.2);
    } else {
      m_intakeSub.intakeRoll.set(0);
    }

    if (opJoy.getBackButton())
      m_intakeSub.intakeRaise.set(0.2);
    else if (opJoy.getStartButton())
      m_intakeSub.intakeRaise.set(-0.6);
  }


  /*
   * Enables elevator PID
   */
  public void enableElevatorPID() {
    m_elevatorPID.enable();
  }

  /*
   * Disables elevator PID
   */
  public void disableElevatorPID() {
    m_elevatorPID.disable();
  }

  // Commands

  // Automated path following

  public Command getAutonomousCommand(String path) {
    switch (path) {
      case "PreloadPath1B": // Launches Cube Ground, AutoDocks
        return new IntakeRelease().alongWith(new AutoBalanceB());
      case "DockPath":
        return pathFollow("output/DockPath.wpilib.json", false);
      case "DockPath2":
        return pathFollow("output/DockPath2.wpilib.json", false);
      case "AutoBalanceDockBack":
        return (new AutoBalanceB());
      case "AutoBalanceDockForward":
        return (new AutoBalanceF());
      case "ConeHigh+Dock":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(new AutoBalanceB());
      case "Cone+Mobility+Dock":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(pathFollow("output/DockPath.wpilib.json", false))
            .andThen(new AutoBalanceF());
      case "CubeMid+Dock":
        return new ParallelRaceGroup(new ElevatorRaiseMid(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new CubeRelease()), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerELevator()), new WaitCommand(0.8))
            .andThen(new AutoBalanceB());
      case "CubeHigh+Dock":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new CubeRelease()), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerELevator()), new WaitCommand(1.2))
            .andThen(new AutoBalanceB());
      case "ConeMid+Dock": // Cone Mid - No Dock
        return new ParallelRaceGroup(new ElevatorRaiseMid(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)));
      case "ConeMid": // Cone Mid + AutoDock
        return new ParallelRaceGroup(new ElevatorRaiseMid(), new WaitCommand(0.5))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new IntakeStow(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)))
            .andThen(new AutoBalanceB());
      case "CubeHigh+Mobility":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)))
            .andThen(pathFollow("output/DockPath.wpilib.json", false));
      case "ConeHigh+Mobility":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)))
            .andThen(pathFollow("output/Dockpath.wpilib.json", false));
      case "Cube+Mobility+Dock":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(pathFollow("output/DockPath.wpilib.json", false))
            .andThen(new AutoBalanceF());
      case "Cube+Engage":
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(pathFollow("output/EngageB.wpilib.json", false));
      case "CubeLow+Mobility":
        return new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2))
            .andThen(pathFollow("output/DockPath.wpilib.json", false));
    }

    return null;
  }

  /**
   * @TODO Auto-generated catch block
   */
  public Command pathFollow(String trajectoryJSON, boolean multiPath) {
    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    } catch (final IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            m_DriveBase::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.m_driveKinematics,
            m_DriveBase::getWheelSpeeds,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            m_DriveBase::voltageControl,
            m_DriveBase);

    if (!multiPath) {
      m_DriveBase.resetOdometry(trajectory.getInitialPose());
    }
    return ramseteCommand;
  }
}
