package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_dist;
  private final double m_speed;

  public DriveDistance(DriveSubsystem drive, double inches, double speed) {
    m_drive = drive;
    m_dist = inches;
    m_speed = speed;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  @Override
  public void end(boolean interuppted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getAverageEncoderDistance()) >= m_dist;
  }
}
