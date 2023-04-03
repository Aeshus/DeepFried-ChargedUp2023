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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonFX m_left1 = new WPI_TalonFX(2);
  private WPI_TalonFX m_right1 = new WPI_TalonFX(0);
  private WPI_TalonFX m_left2 = new WPI_TalonFX(3);
  private WPI_TalonFX m_right2 = new WPI_TalonFX(1);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_left1, m_left2);

  public MotorControllerGroup getLeftMotors() {
    return m_leftMotors;
  }

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_right1, m_right2);

  public MotorControllerGroup getRightMotors() {
    return m_rightMotors;
  }

  private WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  private double leftEncoderPosition;
  private double leftEncoderVelocity;

  private double rightEncoderPosition;
  private double rightEncoderVelocity;

  public DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  public DifferentialDriveOdometry m_odometry;

  public DriveSubsystem() {
    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(false);

    m_right1.setNeutralMode(NeutralMode.Brake);
    m_right2.setNeutralMode(NeutralMode.Brake);
    m_left1.setNeutralMode(NeutralMode.Brake);
    m_left2.setNeutralMode(NeutralMode.Brake);

    m_left1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.drive_FeedbackPidIdx,
        Constants.drive_FeedbackTimeout);
    m_right1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.drive_FeedbackPidIdx,
        Constants.drive_FeedbackTimeout);

    m_odometry = new DifferentialDriveOdometry(getHeading(), leftEncoderPosition, rightEncoderPosition);

    testMotors();

    m_left1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.drive_MotorCurrentLimit, 25,
        Constants.drive_SecondsForOpenRamp));
    m_left2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.drive_MotorCurrentLimit, 25,
        Constants.drive_SecondsForOpenRamp));
    m_right1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.drive_MotorCurrentLimit, 25,
        Constants.drive_SecondsForOpenRamp));
    m_right2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.drive_MotorCurrentLimit, 25,
        Constants.drive_SecondsForOpenRamp));
  }

  /**
   * Make sure the motors are "safe" by testing them against different frames and
   * times
   */
  private void testMotors() {
    m_left1.setStatusFramePeriod(1, 10);
    m_right1.setStatusFramePeriod(1, 10);
    m_left2.setStatusFramePeriod(1, 10);
    m_right2.setStatusFramePeriod(1, 10);

    m_left1.setStatusFramePeriod(3, 13100 * 100);
    m_right1.setStatusFramePeriod(3, 13200 * 100);
    m_left2.setStatusFramePeriod(3, 13500 * 100);
    m_right2.setStatusFramePeriod(3, 13600 * 100);

    m_left1.setStatusFramePeriod(4, 17100 * 100);
    m_right1.setStatusFramePeriod(4, 17200 * 100);
    m_left2.setStatusFramePeriod(4, 17500 * 100);
    m_right2.setStatusFramePeriod(4, 17600 * 100);

    m_left1.setStatusFramePeriod(8, 19100 * 100);
    m_right1.setStatusFramePeriod(8, 19200 * 100);
    m_left2.setStatusFramePeriod(8, 19500 * 100);
    m_right2.setStatusFramePeriod(8, 19600 * 100);

    m_left1.setStatusFramePeriod(14, 23100 * 100);
    m_right1.setStatusFramePeriod(14, 23200 * 100);

    m_left2.setStatusFramePeriod(14, 23500 * 100);
    m_right2.setStatusFramePeriod(14, 23600 * 100);
  }

  @Override
  public void periodic() {
    leftEncoderPosition = m_left1.getSelectedSensorPosition() * Constants.drive_EncoderDistancePerPulse;
    rightEncoderPosition = m_right1.getSelectedSensorPosition() * Constants.drive_EncoderDistancePerPulse;

    leftEncoderVelocity = m_left1.getSelectedSensorVelocity() * Constants.drive_EncoderDistancePerPulse;
    rightEncoderVelocity = m_right1.getSelectedSensorVelocity() * Constants.drive_EncoderDistancePerPulse;

    m_odometry.update(getHeading(), leftEncoderPosition, rightEncoderPosition);
  }

  /**
   * Reset the current position for the motors. Useful for ramsete commands and
   * path management
   */
  public void resetEncoders() {
    m_left1.setSelectedSensorPosition(-0);
    m_left2.setSelectedSensorPosition(-0);
    m_right1.setSelectedSensorPosition(0);
    m_right2.setSelectedSensorPosition(0);
  }

  /**
   * Reset Odometry
   * 
   * @param pose Location of robot
   */
  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getHeading(), this.leftEncoderPosition, this.rightEncoderPosition, pose);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the Gyroscope
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * @return Gyroscope's heading
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  /**
   * @return Gyroscope's pitch
   */
  public double getPitch() {
    return m_gyro.getPitch();
  }

  /**
   * @return Gyroscope's roll
   */
  public double getRoll() {
    return m_gyro.getRoll();
  }

  /**
   * Enables arcade-drive (used by autonomous driving)
   * 
   * @param speed    Normalized speed (-1 -> 1)
   * @param rotation Normalized rotation (-1 -> 1)
   */
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  /**
   * Enables curvature-drive (used by teleoperated driving)
   * 
   * @param speed    Normalzied speed (-1 -> 1)
   * @param rotation Normalized rotation (-1 -> 1)
   * @param turn     Turning in place?
   */
  public void curvatureDrive(double speed, double rotation, boolean turn) {
    m_drive.curvatureDrive(speed, rotation, turn);
  }

  /**
   * Averages the distance between the two encoders
   * 
   * @return Average encoder distance
   */
  public double getAverageEncoderDistance() {
    return (double) (leftEncoderPosition + rightEncoderPosition) / (double) 2;
  }

  /**
   * Gets the speed of the two wheels.
   * <p>
   * m/s
   * 
   * @return Velocity of encoders
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);
  }

  /**
   * Sets voltage for all motors.
   * 
   * @param leftVolts  Voltage for left motors
   * @param rightVolts Voltage for right motors
   */
  public void voltageControl(final double leftVolts, final double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }
}
