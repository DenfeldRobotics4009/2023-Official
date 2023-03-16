package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

import java.util.concurrent.locks.Lock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_PrecisionMode, m_feildOriented;


    PIDController 
        factorController = new PIDController(0.2, 0, 0, 0),
        factorTurnController = new PIDController(0.2, 0, 0, 0);
    double recursiveAdd, recursiveTurnAdd;
    double pidOut, pidTurnOut;

    public DefaultDrive(Drivetrain drivetrainSubsystem,
                               BooleanSupplier precisionMode,
                               BooleanSupplier feildOriented,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier                          
                               ) {
        this.m_feildOriented = feildOriented;
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_PrecisionMode = precisionMode;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);

        factorController.setTarget(0);
        factorTurnController.setTarget(0);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Default Drive", true);

        // PID target will always be 0, so we read the rror
        double target = (m_PrecisionMode.getAsBoolean()) ? 0.5 : 1;
        double turntarget = (m_PrecisionMode.getAsBoolean()) ? 0.35 : 0.75;
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

        double gyroAngle = Autonomous.navxGyro.getAngle() - 90;
        SmartDashboard.putNumber("gyro", gyroAngle);
        // (ysin(theta) - xcos(theta) , ysin(theta) + xcos(theta)) // Relative point of rad theta
    
        double z = m_rotationSupplier.getAsDouble() * pidTurnOut;
        double y = m_translationYSupplier.getAsDouble() * pidOut;
        double x = m_translationXSupplier.getAsDouble() * pidOut;

        if (!m_feildOriented.getAsBoolean()) {
            m_drivetrainSubsystem.drive(
                DeadZoneTuner.adjustForDeadzone(
                    z, 0.3, false), 
                DeadZoneTuner.adjustForDeadzone(
                    x*Math.sin(Math.toRadians(gyroAngle)) - y*Math.cos(Math.toRadians(gyroAngle)), 0.3, false), 
                DeadZoneTuner.adjustForDeadzone(
                    y*Math.sin(Math.toRadians(gyroAngle)) + x*Math.cos(Math.toRadians(gyroAngle)), 0.3, false)
            );
        } else {
            m_drivetrainSubsystem.drive(
                z,
                x,
                y
            );
        }

        // m_drivetrainSubsystem.drive(
        //     m_rotationSupplier.getAsDouble() * pidTurnOut * turn, // Black magic math
        // (m_translationYSupplier.getAsDouble() * pidOut*Math.sin(Math.toRadians(gyroAngle)) + m_translationXSupplier.getAsDouble() * pidOut*Math.cos(Math.toRadians(gyroAngle))),
        // (m_translationXSupplier.getAsDouble() * pidOut*Math.sin(Math.toRadians(gyroAngle)) - m_translationYSupplier.getAsDouble() * pidOut*Math.cos(Math.toRadians(gyroAngle)))
        // );

        // m_drivetrainSubsystem.feildOrientedDrive(
        //     m_rotationSupplier.getAsDouble() * pidTurnOut * turn,
        //     m_translationXSupplier.getAsDouble() * pidOut,
        //     m_translationYSupplier.getAsDouble() * pidOut
        //     // DeadZoneTuner.adjustForDeadzone(
        //     //     m_translationYSupplier.getAsDouble() * pidOut, 
        //     //     0.1 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
        //     //     false)
        // );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0);
        SmartDashboard.putBoolean("Default Drive", false);

    }
}
