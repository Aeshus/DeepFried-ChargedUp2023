// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutoBalanceSubsystem extends SubsystemBase {

  private final BuiltInAccelerometer m_rioAccelerometer = new BuiltInAccelerometer();

  private int m_debounceCount = 0;

  private enum autoState {
    APPROACH_STATION, UP_STATION, ON_STATION, DONE,
  }

  private autoState m_state = autoState.APPROACH_STATION;

  public AutoBalanceSubsystem() {}

  /**
   * Calculates the pitch by doing arctan(-x, (y^2 + z^2)) * (pi / 180)
   * 
   * @return Pitch of built-in accelerometer (radian)
   */
  public double getPitch() {
    return Math.atan2((-m_rioAccelerometer.getX()), Math.sqrt(
        m_rioAccelerometer.getY() * m_rioAccelerometer.getY() + m_rioAccelerometer.getZ() * m_rioAccelerometer.getZ()))
        * 57.3;
  }

  /**
   * Calculates the roll by doing arctan(y, z) * (pi / 180)
   * 
   * @return Roll of built-in accelerometer (radian)
   */
  public double getRoll() {
    return Math.atan2(m_rioAccelerometer.getY(), m_rioAccelerometer.getZ()) * 57.3;
  }

  /**
   * Calculates the tilt by doing sqrt(pitch^2 + roll^2)
   * 
   * @return Tilt of build-in-accelerometer (radian)
   */
  public double getTilt() {
    final double pitch = getPitch();
    final double roll = getRoll();

    if ((pitch + roll) >= 0) {
      return Math.sqrt(pitch * pitch + roll * roll);
    } else {
      return -Math.sqrt(pitch * pitch + roll * roll);
    }
  }

  /**
   * Returns the time in ticks, though multipling it by 50.
   * 
   * @param time Lapsed time
   * @return Time in ticks
   */
  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  /**
   * Does some voodoo magic.
   * 
   * @return Normalized speed (-1 -> 1)
   */
  public double autoBalanceRoutineBackward() {
    switch (m_state) {
    case APPROACH_STATION:
      if (getTilt() > Constants.autob_ChargeStationDegree) {
        m_debounceCount++;
      }

      if (m_debounceCount > secondsToTicks(Constants.autob_DebounceTimeBackward)) {
        m_state = autoState.UP_STATION;
        m_debounceCount = 0;

        return Constants.autob_SpeedSlow;
      }

      return Constants.autob_SpeedFast;
    case UP_STATION:
      if (getTilt() < Constants.autob_LevelDegree) {
        m_debounceCount++;
      }

      if (m_debounceCount > secondsToTicks(Constants.autob_DebounceTimeBackward)) {
        m_state = autoState.ON_STATION;
        m_debounceCount = 0;

        return 0.0;
      }

      return Constants.autob_SpeedSlow;
    case ON_STATION:
      if (Math.abs(getTilt()) <= Constants.autob_LevelDegree / 2) {
        m_debounceCount++;
      }

      if (m_debounceCount > secondsToTicks(Constants.autob_DebounceTimeBackward)) {
        m_state = autoState.DONE;
        m_debounceCount = 0;

        return 0.0;
      }

      if (getTilt() >= Constants.autob_LevelDegree) {
        return -0.1;
      } else if (getTilt() <= -Constants.autob_LevelDegree) {
        return -0.1;
      }

      return 0;

    case DONE:
      return 0.0;
    }

    // If state is somehow null...
    return 0.0;
  }

  /**
   * Does some voodoo magic.
   * 
   * @return Normalized speed (-1 -> 1)
   */
  public double autoBalanceRoutineForward() {
    switch (m_state) {
    case APPROACH_STATION:
      if (getTilt() > Constants.autob_ChargeStationDegree) {
        m_debounceCount++;
      }

      if (m_debounceCount > secondsToTicks(Constants.autob_DebounceTimeForward)) {
        m_state = autoState.UP_STATION;
        m_debounceCount = 0;

        return Constants.autob_SpeedSlow;
      }

      return Constants.autob_SpeedFast;
    case UP_STATION:
      if (getTilt() < Constants.autob_LevelDegree) {
        m_debounceCount++;
      }

      if (m_debounceCount > secondsToTicks(Constants.autob_DebounceTimeForward)) {
        m_state = autoState.ON_STATION;
        m_debounceCount = 0;

        return 0.0;
      }

      return Constants.autob_SpeedSlow;
    case ON_STATION:
      if (Math.abs(getTilt()) <= Constants.autob_LevelDegree / 2) {
        m_debounceCount++;
      }

      if (m_debounceCount > secondsToTicks(Constants.autob_DebounceTimeForward)) {
        m_state = autoState.DONE;
        m_debounceCount = 0;

        return 0.0;
      }

      if (getTilt() >= Constants.autob_LevelDegree) {
        return -0.1;
      } else if (getTilt() <= -Constants.autob_LevelDegree) {
        return -0.1;
      }

      return 0;

    case DONE:
      return 0.0;
    }

    // If state is somehow null
    return 0.0;
  }
}
