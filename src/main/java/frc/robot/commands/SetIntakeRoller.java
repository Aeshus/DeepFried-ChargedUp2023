package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeRoller extends CommandBase {
  private final IntakeSubsystem m_intake;
  private final double m_speed;

  /**
   * Sets the Intake Roller to given speed
   * 
   * @param intake Intake Subsystem
   * @param speed  Speed to set the roller to
   */
  public SetIntakeRoller(IntakeSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = speed;

    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.setRoller(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setRoller(0);
  }
}
