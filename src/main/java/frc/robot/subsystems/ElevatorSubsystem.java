// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ElevatorSubsystem extends ProfiledPIDSubsystem {

  private final CANSparkMax m_rightSpark = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_leftSpark = new CANSparkMax(14, MotorType.kBrushless);

  private final RelativeEncoder m_rightEncoder = m_rightSpark.getEncoder();
  private final RelativeEncoder m_leftEncoder = m_leftSpark.getEncoder();

  public ElevatorSubsystem() {
    super(new ProfiledPIDController(0.07, 0, 0, new TrapezoidProfile.Constraints(200, 200)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    m_rightSpark.set(output);
  }

  public double getRightEncoderPos() {
    return m_rightEncoder.getPosition();
  }

  public double getLeftEncoderPos() {
    return m_leftEncoder.getPosition();
  }

  @Override
  public double getMeasurement() {
    return getRightEncoderPos();
  }

  /**
   * Resets (only?) the right encoder
   */
  public void resetEncoders() {
    m_rightEncoder.setPosition(0);
  }
}
