package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
            DeadZoneTuner.adjustForDeadzone(
                m_rotationSupplier.getAsDouble(), 
                0.01 * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                false),
            DeadZoneTuner.adjustForDeadzone(
                m_translationXSupplier.getAsDouble(), 
                0.01 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
                false),
            DeadZoneTuner.adjustForDeadzone(
                m_translationYSupplier.getAsDouble(), 
                0.01 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
                false)
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0);
    }
}
