package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_PrecisionMode;

    PIDController 
        factorController = new PIDController(0.2, 0, 0, 0),
        factorTurnController = new PIDController(0.2, 0, 0, 0);
    double recursiveAdd, recursiveTurnAdd;
    double pidOut, pidTurnOut;

    public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
                               BooleanSupplier precisionMode,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_PrecisionMode = precisionMode;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        factorController.setTarget(0);
        factorTurnController.setTarget(0);
        // PID target will always be 0, so we read the rror
        double target = (m_PrecisionMode.getAsBoolean()) ? 0.5 : 1;
        double turntarget = (m_PrecisionMode.getAsBoolean()) ? 0.3 : 0.75;
        // Set error as the input
        factorController.setInput(pidOut-(target));
        factorTurnController.setInput(pidTurnOut-(turntarget));
        // Recursively adjust pidOUt
        pidOut += factorController.calculate(0.2, -0.2);
        pidTurnOut += factorTurnController.calculate(0.2, -0.2);
        Drivetrain.clamp(pidOut, 1, 0.5);
        Drivetrain.clamp(pidTurnOut, 0.75, 0.3);

        SmartDashboard.putNumber("pidOUt", pidOut);
        SmartDashboard.putNumber("pidTurnOUt", pidTurnOut);

        m_drivetrainSubsystem.drive(
            DeadZoneTuner.adjustForDeadzone(
                m_rotationSupplier.getAsDouble() * pidTurnOut, 
                0.15 * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                false),
            DeadZoneTuner.adjustForDeadzone(
                m_translationXSupplier.getAsDouble() * pidOut, 
                0.1 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
                false),
            DeadZoneTuner.adjustForDeadzone(
                m_translationYSupplier.getAsDouble() * pidOut, 
                0.1 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
                false)
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0);
    }
}
