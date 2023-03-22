// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {

  public WPI_TalonFX _left1 = new WPI_TalonFX(2);
  public WPI_TalonFX _right1 = new WPI_TalonFX(0);
  public WPI_TalonFX _left2 = new WPI_TalonFX(3);
  public WPI_TalonFX _right2 = new WPI_TalonFX(1);

  public double leftEncPos;
  public double rightEncPos;
  public double leftEncVel;
  public double rightEncVel;

  public DifferentialDrive m_drive = new DifferentialDrive(_left1, _right1);

  public WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  public DifferentialDriveOdometry m_odometry;

  public double robotSpeedFast = 0.4,
      robotSpeedSlow = 0.2,
      onChargeStationDegree = 13.0,
      levelDegree = 6.0,
      debounceTime = 1, // 0.2
      singleTapTime = 0.4, // 0.4
      scoringBackUpTIme = 0.2,
      doubleTapTime = 0.3,
      scoringBackUpTime;
  public int debounceCount = 0, state = 0;

  /** Creates a new DriveBase. */
  public DriveBase() {

    // Sets leader follower relationship
    _left2.follow(_left1);
    _right2.follow(_right1);

    // inverts the left motors
    _left1.setInverted(true);
    _left2.setInverted(true);
    _right1.setInverted(false);
    _right2.setInverted(false);

    // sets motor to brake motor
    _right1.setNeutralMode(NeutralMode.Brake);
    _right2.setNeutralMode(NeutralMode.Brake);
    _left1.setNeutralMode(NeutralMode.Brake);
    _left2.setNeutralMode(NeutralMode.Brake);

    // gets the encoder values
    _left1.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.kDrivePIDIdx, Constants.kDriveTimeoutMs);
    _right1.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.kDrivePIDIdx, Constants.kDriveTimeoutMs);

    m_odometry = new DifferentialDriveOdometry(getHeading(), leftEncPos, rightEncPos);

    // Motor Safety
    _left1.setStatusFramePeriod(1, 10);
    _right1.setStatusFramePeriod(1, 10);
    _left2.setStatusFramePeriod(1, 10);
    _right2.setStatusFramePeriod(1, 10);

    _left1.setStatusFramePeriod(3, 13100 * 100);
    _right1.setStatusFramePeriod(3, 13200 * 100);
    _left2.setStatusFramePeriod(3, 13500 * 100);
    _right2.setStatusFramePeriod(3, 13600 * 100);

    _left1.setStatusFramePeriod(4, 17100 * 100);
    _right1.setStatusFramePeriod(4, 17200 * 100);
    _left2.setStatusFramePeriod(4, 17500 * 100);
    _right2.setStatusFramePeriod(4, 17600 * 100);

    _left1.setStatusFramePeriod(8, 19100 * 100);
    _right1.setStatusFramePeriod(8, 19200 * 100);
    _left2.setStatusFramePeriod(8, 19500 * 100);
    _right2.setStatusFramePeriod(8, 19600 * 100);

    _left1.setStatusFramePeriod(14, 23100 * 100);
    _right1.setStatusFramePeriod(14, 23200 * 100);
    ;
    _left2.setStatusFramePeriod(14, 23500 * 100);
    _right2.setStatusFramePeriod(14, 23600 * 100);

    _left1.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            true, Constants.CurrentLimit, 25, Constants.secondsForOpenRamp));
    _left2.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            true, Constants.CurrentLimit, 25, Constants.secondsForOpenRamp));
    _right1.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            true, Constants.CurrentLimit, 25, Constants.secondsForOpenRamp));
    _right2.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            true, Constants.CurrentLimit, 25, Constants.secondsForOpenRamp));
  }

  @Override
  public void periodic() {

    leftEncPos = _left1.getSelectedSensorPosition() * Constants.kEncoderDistancePerPulse;
    rightEncPos = _right1.getSelectedSensorPosition() * Constants.kEncoderDistancePerPulse;
    leftEncVel = _left1.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;
    rightEncVel = _right1.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;

    m_odometry.update(getHeading(), leftEncPos, rightEncPos);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncVel, rightEncVel);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetEncoders() {
    _left1.setSelectedSensorPosition(-0);
    _left2.setSelectedSensorPosition(-0);
    _right1.setSelectedSensorPosition(0);
    _right2.setSelectedSensorPosition(0);
  }

  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getHeading(), leftEncPos, rightEncPos, pose);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  // public double getHeadingActual() {
  //   return -m_gyro.getYaw() + Math.toDegrees(initTheta());
  // }`

  public void voltageControl(final double leftVolts, final double rightVolts) {
    _left1.setVoltage(leftVolts);
    _right1.setVoltage(rightVolts);
    m_drive.feed();

    // used to be negative
  }
  // This method will be called once per scheduler run
}
