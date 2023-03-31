// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class InvertMotors extends CommandBase {
  private DriveSubsystem m_drive;

  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  public InvertMotors(DriveSubsystem driveSubsystem) {
    m_drive = driveSubsystem;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_leftMotors = m_drive.getLeftMotors();
    m_rightMotors = m_drive.getRightMotors();
  }

  @Override
  public void execute() {
    m_leftMotors.setInverted(!m_leftMotors.getInverted());
    m_rightMotors.setInverted(!m_rightMotors.getInverted());
  }
}
