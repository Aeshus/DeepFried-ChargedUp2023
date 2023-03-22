// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class InvertMotors extends CommandBase {
  /** Creates a new InvertMotors. */
  public InvertMotors() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.m_DriveBase._left1.setInverted(false);
    Robot.m_robotContainer.m_DriveBase._left2.setInverted(false);
    Robot.m_robotContainer.m_DriveBase._right1.setInverted(true);
    Robot.m_robotContainer.m_DriveBase._right2.setInverted(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
