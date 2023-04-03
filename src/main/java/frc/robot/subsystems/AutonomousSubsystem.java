package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceBackward;
import frc.robot.commands.AutoBalanceForward;
import frc.robot.commands.SetIntakeRaise;
import frc.robot.commands.SetIntakeRoller;
import frc.robot.commands.SetElevatorHeight;

public class AutonomousSubsystem extends SubsystemBase {

  private enum AutonomousPath {
    DOCK_FORWARDS, DOCK_BACKWARDS, MOBILITY_PATH_BACKWARDS, MOBILITY_PATH_FORWARDS,

    CONE_MID, CONE_MID_DOCK, CONE_HIGH_DOCK, CONE_HIGH_MOBILITY, CONE_MOBILITY_DOCK,

    CUBE_LOW_AUTO_DOCK, CUBE_LOW_MOBILITY, CUBE_MID_DOCK, CUBE_HIGH_DOCK, CUBE_HIGH_ENGAGE, CUBE_HIGH_MOBILITY,
    CUBE_MOBILITY_DOCK,
  }

  private DriveSubsystem m_drive;
  private AutoBalanceSubsystem m_autoBalance;
  private ElevatorSubsystem m_elevator;
  private IntakeSubsystem m_intake;

  public AutonomousSubsystem(DriveSubsystem drive, AutoBalanceSubsystem autoBalance, ElevatorSubsystem elevator,
      IntakeSubsystem intake) {

    m_drive = drive;
    m_autoBalance = autoBalance;
    m_elevator = elevator;
    m_intake = intake;
  }

  /**
   * Gets list of command names
   * 
   * @return String array of all commands
   */
  public String[] getCommands() {
    return Arrays.stream(AutonomousPath.values()).map(AutonomousPath::name).toArray(String[]::new);
  }

  /**
   * Matches command name to the actual command
   * 
   * @param path Name of path/command
   * @return Command to follow path
   */
  public Command getAutonomousCommand(String path) {
    switch (getPathFromName(path)) {
    case DOCK_FORWARDS:
      return (new AutoBalanceForward(m_autoBalance, m_drive));
    case DOCK_BACKWARDS:
      return (new AutoBalanceBackward(m_autoBalance, m_drive));

    case MOBILITY_PATH_BACKWARDS:
      return pathFollow("output/DockPath.wpilib.json", false);
    case MOBILITY_PATH_FORWARDS:
      return pathFollow("output/DockPath2.wpilib.json", false);

    case CONE_MID:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightMid),
          new WaitCommand(0.5))
              .andThen(
                  new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_lower), new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_ConeRelease),
                  new WaitCommand(0.8)))
              .andThen(new SetIntakeRaise(m_intake, Constants.intake_stow), new WaitCommand(0.8))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(0.8)))
              .andThen(new AutoBalanceBackward(m_autoBalance, m_drive));
    case CONE_MID_DOCK:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightMid),
          new WaitCommand(0.8))
              .andThen(
                  new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_lower), new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_ConeRelease),
                  new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_stow), new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(0.8)));
    case CONE_HIGH_DOCK:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(1.2))
              .andThen(
                  new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_lower), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_ConeRelease),
                  new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_stow), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(1.2)))
              .andThen(new AutoBalanceBackward(m_autoBalance, m_drive));
    case CONE_HIGH_MOBILITY:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(0.8))
              .andThen(
                  new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_lower), new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_ConeRelease),
                  new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_stow), new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(0.8)))
              .andThen(pathFollow("output/Dockpath.wpilib.json", false));
    case CONE_MOBILITY_DOCK:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(1.2))
              .andThen(
                  new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_lower), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_ConeRelease),
                  new WaitCommand(0.8)))
              .andThen(new ParallelRaceGroup(new SetIntakeRaise(m_intake, Constants.intake_stow), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(1.2)))
              .andThen(pathFollow("output/DockPath.wpilib.json", false))
              .andThen(new AutoBalanceForward(m_autoBalance, m_drive));

    case CUBE_LOW_AUTO_DOCK:
      return new SetIntakeRoller(m_intake, Constants.intake_release)
          .alongWith(new AutoBalanceBackward(m_autoBalance, m_drive));
    case CUBE_LOW_MOBILITY:
      return new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease), new WaitCommand(1.2))
          .andThen(pathFollow("output/DockPath.wpilib.json", false));
    case CUBE_MID_DOCK:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightMid),
          new WaitCommand(0.8)).andThen(
              new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease), new WaitCommand(0.8))
                  .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow)),
                      new WaitCommand(0.8))
                  .andThen(new AutoBalanceBackward(m_autoBalance, m_drive)));
    case CUBE_HIGH_DOCK:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(0.8)).andThen(
              new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease), new WaitCommand(0.8))
                  .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow)),
                      new WaitCommand(1.2))
                  .andThen(new AutoBalanceBackward(m_autoBalance, m_drive)));
    case CUBE_HIGH_ENGAGE:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(1.2)).andThen(
              new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(1.2)))
              .andThen(pathFollow("output/EngageB.wpilib.json", false));
    case CUBE_HIGH_MOBILITY:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(0.8)).andThen(
              new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(0.8)))
              .andThen(pathFollow("output/DockPath.wpilib.json", false));
    case CUBE_MOBILITY_DOCK:
      return new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightHigh),
          new WaitCommand(1.2)).andThen(
              new ParallelRaceGroup(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease), new WaitCommand(1.2)))
              .andThen(new ParallelRaceGroup(new SetElevatorHeight(m_elevator, Constants.elevator_HeightLow),
                  new WaitCommand(1.2)))
              .andThen(pathFollow("output/DockPath.wpilib.json", false))
              .andThen(new AutoBalanceForward(m_autoBalance, m_drive));
    default:
      return null;
    }
  }

  /**
   * Gets AutonomousPath enum from it's name
   * 
   * @param name Name of path/command
   * @return AutonomousPath enum
   */
  public AutonomousPath getPathFromName(String name) {
    return AutonomousPath.valueOf(name);
  }

  /**
   * Gets command to follow path using a file-path.
   * 
   * @param path      Filesystem path to .json file
   * @param multiPath If this is multiple paths (never used)
   * @return Ramsette Command
   */
  public Command pathFollow(String path, boolean multiPath) {
    Trajectory trajectory = new Trajectory();

    try {
      Path trajectoryFile = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFile);
    } catch (Exception e) {
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drive::getPose,
        new RamseteController(Constants.auton_RamseteConvergence, Constants.auton_RamseteDamping),
        new SimpleMotorFeedforward(Constants.auton_StaticGain, Constants.auton_VelocityGain,
            Constants.auton_AccelerationGain),
        Constants.auton_DiffDriveKinematics, m_drive::getWheelSpeeds, new PIDController(1, 0, 0),
        new PIDController(1, 0, 0), m_drive::voltageControl, m_drive);

    if (!multiPath) {
      m_drive.resetOdometry(trajectory.getInitialPose());
    }

    return ramseteCommand;
  }

  /**
   * Reset a bunch of stuff for autonomous driving
   */
  public void autonomousInit() {
    m_drive.resetGyro();
    m_drive.resetEncoders();
    m_elevator.resetEncoders();
  }
}
