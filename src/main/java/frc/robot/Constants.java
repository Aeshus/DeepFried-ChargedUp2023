// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

  /// Drive Subsystem ///

  /**
   * The maximum current for the motors driving the robot.
   * <p>
   * (amps)
   */
  public static final double drive_MotorCurrentLimit = 50;

  /**
   * Amount of time before the voltage starts being limited.
   * <p>
   * (seconds)
   */
  public static final double drive_SecondsForOpenRamp = 1;

  /**
   * Timeout for the feedback sensor.
   * <p>
   * (miliseconds)
   */
  public final static int drive_FeedbackTimeout = 30;

  /**
   * Index for the feedback sensor.
   * <ul>
   * <li>0 - Primary Closed-Loop</li>
   * <li>1 - Auxillery Closed-Loop</li>
   * </ul>
   */
  public final static int drive_FeedbackPidIdx = 0;

  /**
   * Calculates the encoder distance using the distance-per-pulse and multiplying
   * by current position.
   * <p>
   * (4 * pi 2.54) / (100 * 2048 * 9.92) ??
   */
  public static final double drive_EncoderDistancePerPulse = (4 * Math.PI * 2.54 * 1) / (100.0 * 2048 * 9.9200);

  /// Autonomous Subsystem ///

  /**
   * Static Gain for the Autonomous Ramsete Command.
   * <p>
   * (volts)
   */
  public static final double auton_StaticGain = 0.18531;

  /**
   * Velocity Gain for the Autonomous Ramsete Command.
   * <p>
   * (volts * seconds / meters)
   */
  public static final double auton_VelocityGain = 1.0502;

  /**
   * Acceleration Gain for the Autonomous Ramsete Command.
   * <p>
   * (volt * seconds^2 / meters)
   */
  public static final double auton_AccelerationGain = 0.13501;

  /**
   * Track width used for the Autonomous Differential Drive Kinematics.
   * <p>
   * (meters)
   */
  public static final double auton_DifferentialTrackWidth = 0.514;

  /**
   * Tuning parameter "b" for Ramsete Unicycle Controller.
   * <p>
   * Larger values make convergence more aggressive.
   */
  public static final double auton_RamseteConvergence = 2;

  /**
   * Tuning parameter "zeta" for Ramsete Unicycle Controller.
   * <p>
   * Larger values provide more damping.
   */
  public static final double auton_RamseteDamping = 0.7;

  /**
   * Differential Drive Kinmeatics used by Ramsete Controller
   */
  public static final DifferentialDriveKinematics auton_DiffDriveKinematics = new DifferentialDriveKinematics(
      auton_DifferentialTrackWidth);

  // AutoBalanceSubsystem
  public final static double autob_SpeedFast = 0.4;
  public final static double autob_SpeedSlow = 0.2;
  public final static double autob_ChargeStationDegree = 13.0;
  public final static double autob_LevelDegree = 6.0;
  public final static double autob_DebounceTimeBackward = 2.25;
  public final static double autob_DebounceTimeForward = 0.1;
  public final static double autob_SingleTapTime = 0.4;
  public final static double autob_ScoringBackUpTime = 0.2;
  public final static double autob_DoubleTapTime = 0.3;

  // IntakeSubsystem

  public final static double intake_ConeRelease = 0.5;
  public final static double intake_CubeRelease = 0.5;

  // In Amps
  public final static int intake_RaiseCurrent = 35;
  public final static int intake_RollCurrent = 35;

  public final static double intake_lower = 0.6;
  public final static double intake_stow = -0.5;
  public final static double intake_release = -0.8;

  // Elevator subsystem
  public final static double elevator_HeightHigh = -50;
  public final static double elevator_HeightMid = -20;
  public final static double elevator_HeightLow = 0;
}
