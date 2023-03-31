// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeRaise = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax intakeRoll = new CANSparkMax(21, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeRaise.setSmartCurrentLimit(Constants.intakeRaiseCurrent);
    intakeRoll.setSmartCurrentLimit(Constants.intakeRollCurrent);

    intakeRaise.setIdleMode(IdleMode.kBrake);
  }

  public void setRoller(double speed) {
    intakeRoll.set(speed);
  }

  public void setRaise(double speed) {
    intakeRaise.set(speed);
  }

}
