// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class AutoCommand extends CommandBase {

  Timer m_timer = new Timer();

  Drivetrain m_drive;
  PathPlannerTrajectory m_path;

  /** Creates a new Autonomous. */
  public AutoCommand(
    Drivetrain DriveSubsystem, 
    PathPlannerTrajectory PathPlannerPath
  ) {
    addRequirements(DriveSubsystem);

    m_drive = DriveSubsystem;
    m_path = PathPlannerPath;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Cast due to PathPlannerState functioning as Trajectory.State
    PathPlannerState m_state = (PathPlannerState) m_path.sample(m_timer.get());

    m_drive.drive(
      m_state.angularVelocityRadPerSec, 
      // This may be false, if it is, use m_state.holonomicRotation funny buiseness
      0, // Zero, as PathPlanner always drives in the direction the robot is facing
      m_state.velocityMetersPerSecond
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_path.getTotalTimeSeconds();
  }
}
