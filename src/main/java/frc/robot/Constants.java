// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

  // DriveSubsystem
  public static final double CurrentLimit = 50;
  public static final double secondsForOpenRamp = 1; // 0.8
  public final static int kDriveTimeoutMs = 30,
      kDrivePIDIdx = 0;

  // DriveBase Constants
  public static final double ksVolts = 0.18531, // 0.65634,
      kvVoltSecondsPerMeter = 1.0502, // 2.6376, //0.1106,
      kaVoltSecondsSquaredPerMeter = 0.13501, // 1.15 //0.095387,
      kTrackwidthMeters = 0.514, // ChargedUp Update
      kP = 1, // 0.17833,
      kD = 0.0,
      kMaxSpeedMetersPerSecond = 4.6634, // ChargedUp
      kMaxAccelerationMetersPerSecondSquared = 5,
      kRamseteB = 2,
      kRamseteZeta = 0.7;

  public static final double kEncoderDistancePerPulse = (4 * Math.PI * 2.54 * 1) / (100.0 * 2048 * 9.9200);

  public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
      kTrackwidthMeters);

  // AutoBalanceSubsystem
  public final static double robotSpeedFast = 0.4,
      robotSpeedSlow = 0.2,
      onChargeStationDegree = 13.0,
      levelDegree = 6.0,
      debounceTimeBackward = 2.25,
      debounceTimeForward = 0.1,
      singleTapTime = 0.4,
      scoringBackUpTime = 0.2,
      doubleTapTime = 0.3;

  // IntakeSubsystem

  public final static double coneRelease = 0.5,
      cubeRelease = 0.5;

  // In Amps
  public final static int intakeRaiseCurrent = 35,
      intakeRollCurrent = 35;

  // Elevator subsystem
  public final static double elevatorHigh = -50,
      elevatorMid = -20,
      elevatorLow = 0;

  public final static double lowerIntake = 0.6,
      stowIntake = -0.5;

  public final static double releaseIntake = -0.8;

}
