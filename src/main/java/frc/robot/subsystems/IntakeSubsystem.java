// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Manages the intake system. It's the thing sucking-in and spitting-out foam objects!
 * @author Aeshus
 * @version 1.0; 
 */

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
    intakeRaise.setSmartCurrentLimit(Constants.intake_RaiseCurrent);
    intakeRoll.setSmartCurrentLimit(Constants.intake_RollCurrent);

    intakeRaise.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the roller's speed
   * <p>
   * (unit/seconds)
   * 
   * @param speed Speed of roller
   */
  public void setRoller(double speed) {
    intakeRoll.set(speed);
  }

  /**
   * Sets the raise's speed
   * <p>
   * (unit/seconds)
   * 
   * @param speed Speed of raise
   */
  public void setRaise(double speed) {
    intakeRaise.set(speed);
  }

}
