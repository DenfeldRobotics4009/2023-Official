// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightServer;

public class AimTowardsTarget extends CommandBase {
  Autonomous m_auto;

  PIDController TurnController = new PIDController(0.0025, 0, 0.00025, 0);
  PIDController DriveController = new PIDController(0.04, 0, 0, 0);

  DoubleSupplier m_translationYSupplier, m_translationXSupplier;

  /**
   * Turns towards the current limelight target, while
   * still allowing x and y movement.
   * @param AutoSubsystem
   * @param dummyDrive Drivetrain subsystem to trick WPILib into
   * stopping the default drive command
   * @param translationXSupplier jsX
   * @param translationYSupplier jsY
   */
  public AimTowardsTarget(
    Autonomous AutoSubsystem, Drivetrain dummyDrive,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier
  ) {
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;

    addRequirements(AutoSubsystem, dummyDrive);
    m_auto = AutoSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TurnController.setTolerance(1);
    TurnController.setTarget(0.5);

    DriveController.setTarget(Constants.LimelightDegreesTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriveController.setInput(
      // Set default input as the target, to make calculate return 0. This also applied to TurnController
      LimelightServer.ty.getDouble(Constants.LimelightDegreesTarget)
    );

    m_auto.turn(
      // Negative, as this is to calculate which way to turn to turn towards the target.
      -LimelightServer.tx.getDouble(0), 
      TurnController, 
      0.5,

      // Supply standard jsx and jsy for driving while aiming
      DeadZoneTuner.adjustForDeadzone(
        m_translationXSupplier.getAsDouble(), 
        0.1 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
        false),
      DriveController.calculate(1, -1)
        * Constants.MAX_VELOCITY_METERS_PER_SECOND
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
