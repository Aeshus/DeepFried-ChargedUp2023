package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeight extends CommandBase {
  private final ElevatorSubsystem m_elevator;
  private final double m_height;

  /**
   * Sets elevator height
   * 
   * @param elevatorSubsystem Elevator Subystem
   * @param height            Height of elevator
   */
  public SetElevatorHeight(ElevatorSubsystem elevatorSubsystem, double height) {
    m_elevator = elevatorSubsystem;
    m_height = height;

    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.enable();
  }

  @Override
  public void execute() {
    m_elevator.setGoal(m_height);
  }

  @Override
  public void end(boolean interupted) {
    if (interupted) {
      m_elevator.setGoal(m_elevator.getMeasurement());
    }

    m_elevator.disable();
  }

  @Override
  public boolean isFinished() {
    return (m_elevator.getMeasurement() == m_height);
  }
}
