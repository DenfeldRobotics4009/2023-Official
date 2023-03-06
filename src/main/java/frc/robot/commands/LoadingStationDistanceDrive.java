// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class LoadingStationDistanceDrive extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_PrecisionMode;

    PIDController 
        factorController = new PIDController(0.2, 0, 0, 0),
        factorTurnController = new PIDController(0.2, 0, 0, 0),
        DriveController = new PIDController(0.001, 0, 0, 0);

    double recursiveAdd, recursiveTurnAdd;
    double pidOut, pidTurnOut;

    public LoadingStationDistanceDrive(Drivetrain drivetrainSubsystem,
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

        factorController.setTarget(0);
        factorTurnController.setTarget(0);

        DriveController.setTarget(Constants.LoadingStationDistGoal);
        DriveController.setTolerance(1);
    }

    @Override
    public void execute() {

        double distance = Arm.distSensor.GetRange();

        // PID target will always be 0, so we read the rror
        double target = (m_PrecisionMode.getAsBoolean()) ? 0.5 : 1;
        double turntarget = (m_PrecisionMode.getAsBoolean()) ? 0.5 : 0.75;
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

        SmartDashboard.putNumber("dist", distance);

        if (distance == -1) {
          m_drivetrainSubsystem.drive(
              m_rotationSupplier.getAsDouble() * pidTurnOut,
              m_translationXSupplier.getAsDouble() * pidOut,
              m_translationYSupplier.getAsDouble() * pidOut
          );
        } else {
          DriveController.setInput(distance);

          m_drivetrainSubsystem.drive(
              m_rotationSupplier.getAsDouble() * pidTurnOut,
              m_translationXSupplier.getAsDouble() * pidOut,
              DriveController.calculate(
                Constants.MAX_VELOCITY_METERS_PER_SECOND, 
                -Constants.MAX_VELOCITY_METERS_PER_SECOND
              )
          );
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0);
    }
}
