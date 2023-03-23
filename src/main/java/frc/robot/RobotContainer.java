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

/*
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

  /** Type for held object of Roller */
  public enum RollerState {
    CONE,
    CUBE,
    NEITHER
  }

  public enum PathType {
    DOCK_FORWARDS,
    DOCK_BACKWARDS,
    MOBILITY_PATH_BACKWARDS,
    MOBILITY_PATH_FORWARDS,

    CONE_MID,
    CONE_MID_DOCK,
    CONE_HIGH_DOCK,
    CONE_HIGH_MOBILITY,
    CONE_MOBILITY_DOCK,

    CUBE_LOW_AUTO_DOCK,
    CUBE_LOW_MOBILITY,
    CUBE_MID_DOCK,
    CUBE_HIGH,
    CUBE_HIGH_DOCK,
    CUBE_HIGH_ENGAGE,
    CUBE_HIGH_MOBILITY,
    CUBE_MOBILITY_DOCK,
  }

  // Last held object by Roller
  RollerState lastHeld;

  /**
   * Normalized magnitude for a given axis of xbox Controller.
   *
   * <p>Axes: 1. LX Axis 2. LY Axis 3. L Trigger 4. R Trigger 5. RX Axis 6. RY Axis
   *
   * @param axis joystick Axis
   * @return range between 1-0.
   */
  public double getDriveJoy(int axis) {
    return normalizeJoy(driveJoy.getRawAxis(axis));
  }

  /**
   * Gets the absolute value of the raw input (range -1 -> 1 becomes 0 -> 1). Then ignores any
   * magnitudes less than 0.1 (dead-zone).
   *
   * @param raw joystick value
   * @return axis magnitude
   */
  private double normalizeJoy(double raw) {
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }

  /**
   * Magnitude of the x-axis of right joystick. Used for Rotation
   *
   * @return magnitude
   */
  public double getDriveJoyXR() {
    return getDriveJoy(4) / 2.5;
  }

  /**
   * Magnitude of the y-axis of left joystick. Used for Forward
   *
   * @return magnitude
   */
  public double getDriveJoyYL() {
    return getDriveJoy(1);
  }

  /**
   * Null constructor Simply builds class as a container and runs the individual *Init() functions
   * from Robot class.
   */
  public RobotContainer() {}

  /** Initialize Smart-dashboard Automatically populated by the PathType enum. */
  public void roboInit() {

    // Iterate over PathType enum and automatically populate options
    for (PathType path : PathType.values()) {
      autoChooser.addOption(path.name().replaceAll("_", " ").toLowerCase(), path.name());
    }

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  /** Initializes autonomous control */
  public void autoInit() {
    // Reset everything
    m_DriveBase.m_gyro.reset();
    m_DriveBase.resetEncoders();
    m_elevatorPID.encoderR.setPosition(0);

    // Follow the selected path.
    if (autoChooser.getSelected() != null) {
      m_autonomousCommand = getAutonomousCommand(autoChooser.getSelected());
      m_autonomousCommand.schedule();
    }
  }

  /** Polls autonomous events (Tilt) */
  public void autoPeriodic() {
    m_autoBalance.getTilt();
    SmartDashboard.putNumber("Tilt", m_autoBalance.getTilt());
  }

  /** Resets device and allows for remote control */
  public void teleOperatedInit() {
    m_DriveBase.resetEncoders();
    lastHeld = RollerState.NEITHER;
  }

  /** Polls for inputs and executes any found. */
  public void teleoperatedPeriodic() {
    SmartDashboard.putNumber("Encoder Right", m_elevatorPID.encoderR.getPosition());

    // Use Left-Y axis for driving, Right-X axis for rotation
    m_DriveBase.m_drive.curvatureDrive(-getDriveJoyYL(), -getDriveJoyXR(), true);

    // Elevator Set-points - Magic Numbers?
    final double setpoint1 = -20;
    final double setpoint2 = -54;
    final double setpoint3 = -62;

    // Elevator Buttons
    if (opJoy.getAButton()) m_elevatorPID.setGoal(0);
    else if (opJoy.getBButton()) m_elevatorPID.setGoal(setpoint1);
    else if (opJoy.getYButton()) m_elevatorPID.setGoal(setpoint2);
    else if (opJoy.getXButton()) m_elevatorPID.setGoal(setpoint3);

    // Idk if one of the elevator buttons wasn't supposed to do this?
    enableElevatorPID();

    // Roller Buttons
    if (opJoy.getRightBumper()) {
      m_intakeSub.intakeRoll.set(1);
      lastHeld = RollerState.CONE;
    } else if (opJoy.getLeftBumper()) {
      m_intakeSub.intakeRoll.set(-0.8);
      lastHeld = RollerState.CUBE;
    } else if (lastHeld == RollerState.CONE) {
      m_intakeSub.intakeRoll.set(0.2);
    } else if (lastHeld == RollerState.CUBE) {
      m_intakeSub.intakeRoll.set(-0.2);
    } else {
      m_intakeSub.intakeRoll.set(0);
    }

    if (opJoy.getBackButton()) m_intakeSub.intakeRaise.set(0.2);
    else if (opJoy.getStartButton()) m_intakeSub.intakeRaise.set(-0.6);
  }

  /** Enables elevator PID */
  public void enableElevatorPID() {
    m_elevatorPID.enable();
  }

  /** Disables elevator PID */
  public void disableElevatorPID() {
    m_elevatorPID.disable();
  }

  /**
   * Give command to follow path depending on the path name given. It matches the string to the enum
   * and does the case for each one.
   *
   * <p>Can return null...
   *
   * <p>TODO: Replace with Optional<Command>
   *
   * @param option name
   * @return Command to follow for path
   */
  public Command getAutonomousCommand(String command) {
    switch (PathType.valueOf(command)) {
      case DOCK_FORWARDS:
        return (new AutoBalanceF());
      case DOCK_BACKWARDS:
        return (new AutoBalanceB());

      case MOBILITY_PATH_BACKWARDS:
        return pathFollow("output/DockPath.wpilib.json", false);
      case MOBILITY_PATH_FORWARDS:
        return pathFollow("output/DockPath2.wpilib.json", false);

      case CONE_MID: // ??
        return new ParallelRaceGroup(new ElevatorRaiseMid(), new WaitCommand(0.5))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new IntakeStow(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)))
            .andThen(new AutoBalanceB());
      case CONE_MID_DOCK: // ??
        return new ParallelRaceGroup(new ElevatorRaiseMid(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)));
      case CONE_HIGH_DOCK:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(new AutoBalanceB());
      case CONE_HIGH_MOBILITY:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)))
            .andThen(pathFollow("output/Dockpath.wpilib.json", false));
      case CONE_MOBILITY_DOCK:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new LowerIntake(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new ConeRelease(), new WaitCommand(0.8)))
            .andThen(new ParallelRaceGroup(new IntakeStow(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(pathFollow("output/DockPath.wpilib.json", false))
            .andThen(new AutoBalanceF());

      case CUBE_LOW_AUTO_DOCK:
        return new IntakeRelease().alongWith(new AutoBalanceB());
      case CUBE_LOW_MOBILITY:
        return new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2))
            .andThen(pathFollow("output/DockPath.wpilib.json", false));
      case CUBE_MID_DOCK:
        return new ParallelRaceGroup(new ElevatorRaiseMid(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new CubeRelease()), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerELevator()), new WaitCommand(0.8))
            .andThen(new AutoBalanceB());
      case CUBE_HIGH_DOCK:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new CubeRelease()), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new LowerELevator()), new WaitCommand(1.2))
            .andThen(new AutoBalanceB());
      case CUBE_HIGH_ENGAGE:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(pathFollow("output/EngageB.wpilib.json", false));
      case CUBE_HIGH_MOBILITY:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(0.8))
            .andThen(new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(0.8)))
            .andThen(pathFollow("output/DockPath.wpilib.json", false));
      case CUBE_MOBILITY_DOCK:
        return new ParallelRaceGroup(new ElevatorRaiseTop(), new WaitCommand(1.2))
            .andThen(new ParallelRaceGroup(new CubeRelease(), new WaitCommand(1.2)))
            .andThen(new ParallelRaceGroup(new LowerELevator(), new WaitCommand(1.2)))
            .andThen(pathFollow("output/DockPath.wpilib.json", false))
            .andThen(new AutoBalanceF());
    }

    return null;
  }

  /**
   * Reads given file and executes path.
   *
   * <p>Can return null if can't read path...
   *
   * @param trajectoryJSON file for path (which encoded in JSON)
   * @param multiPath ??
   * @return Path command
   */
  public Command pathFollow(String trajectoryJSON, boolean multiPath) {
    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    } catch (final IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      return null;
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
