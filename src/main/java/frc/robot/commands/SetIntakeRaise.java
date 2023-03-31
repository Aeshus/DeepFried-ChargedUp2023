package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeRaise extends CommandBase {
  private final IntakeSubsystem m_intake;
  private final double m_speed;

  public SetIntakeRaise(IntakeSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = speed;

    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.setRaise(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setRaise(0);
  }
}
