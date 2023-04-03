package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_zRotation;

    /**
     * Sets the default driving mode
     * 
     * @param drive    Drive Subsystem
     * @param speed    Motor Speed
     * @param rotation Rotation
     */
    public DefaultDrive(DriveSubsystem drive, DoubleSupplier speed, DoubleSupplier rotation) {
        m_drive = drive;
        m_xSpeed = speed;
        m_zRotation = rotation;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.curvatureDrive(m_xSpeed.getAsDouble(), m_zRotation.getAsDouble(), true);
    }

}
