// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.SetIntakeRoller;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.subsystems.AutoBalanceSubsystem;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {

  private final AutoBalanceSubsystem m_autoBalance = new AutoBalanceSubsystem();
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final AutonomousSubsystem m_auto = new AutonomousSubsystem(m_drive, m_autoBalance, m_elevator, m_intake);

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  private SendableChooser<String> autoChooser;

  public RobotContainer(SendableChooser<String> autoChooser) {
    this.autoChooser = autoChooser;

    setDefaultKeybinds();

    m_drive.setDefaultCommand(
        new DefaultDrive(m_drive, () -> -m_driverController.getLeftY(), () -> -m_driverController.getRightX()));

    setupDashboard();
  }

  /**
   * Initialize the smart dashboard and auto chooser.
   * <p>
   * It iterates over all the avalible autonomous commands and populates the smart
   * dashboard.
   */
  private void setupDashboard() {
    Arrays.stream(m_auto.getCommands())
        .forEach(name -> autoChooser.addOption(name.replaceAll("_", " ").toLowerCase(), name));

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  /**
   * Setup the default keybinds
   */
  private void setDefaultKeybinds() {
    // onTrue vs. toggleOnTrue?

    new JoystickButton(m_operatorController, Button.kA.value).onTrue(new SetElevatorHeight(m_elevator, 0));
    new JoystickButton(m_operatorController, Button.kB.value).onTrue(new SetElevatorHeight(m_elevator, -20));
    new JoystickButton(m_operatorController, Button.kY.value).onTrue(new SetElevatorHeight(m_elevator, -54));
    new JoystickButton(m_operatorController, Button.kX.value).onTrue(new SetElevatorHeight(m_elevator, -60));

    new JoystickButton(m_operatorController, Button.kRightBumper.value).onTrue(new SetIntakeRoller(m_intake, 1))
        .onFalse(new SetIntakeRoller(m_intake, Constants.intake_ConeRelease));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value).onTrue(new SetIntakeRoller(m_intake, -0.8))
        .onFalse(new SetIntakeRoller(m_intake, Constants.intake_CubeRelease));
  }

  /**
   * Get the autonomous command from the Smart Dashboard's name
   * 
   * @param path Name of the path
   * @return Command
   */
  public Command getAutonomousCommand(String path) {
    return m_auto.getAutonomousCommand(path);
  }

  /**
   * Initialize the autonomous system
   */
  public void autonomousInit() {
    m_auto.autonomousInit();
  }

  /**
   * Initialize the teloperated system (manual control)
   */
  public void teleOperatedInit() {
    m_drive.resetEncoders();
  }

}
