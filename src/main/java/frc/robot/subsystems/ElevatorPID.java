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

public class ElevatorPID extends ProfiledPIDSubsystem {

  public final CANSparkMax rightSpark = new CANSparkMax(15, MotorType.kBrushless);
  public final CANSparkMax leftSpark = new CANSparkMax(14, MotorType.kBrushless);
  public final RelativeEncoder encoderR = rightSpark.getEncoder();
  public final RelativeEncoder encoderL = leftSpark.getEncoder();

  /** Creates a new ElevatorPID. */
  public ElevatorPID() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.07,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(200, 200)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    rightSpark.set(output);
  }

  public double getEncoderPos() {
    return encoderR.getPosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderPos();
  }
}
