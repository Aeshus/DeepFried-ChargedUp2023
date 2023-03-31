// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoBalanceSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceForward extends CommandBase {
  private final AutoBalanceSubsystem m_autoBalance;
  private final DriveSubsystem m_drive;

  public AutoBalanceForward(AutoBalanceSubsystem autoBalance, DriveSubsystem drive) {
    m_autoBalance = autoBalance;
    m_drive = drive;

    addRequirements(m_autoBalance, m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double speed = m_autoBalance.autoBalanceRoutineForward();
    m_drive.arcadeDrive(speed, 0);
  }

}
