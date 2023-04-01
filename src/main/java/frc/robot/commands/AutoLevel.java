// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends CommandBase {


  PIDController driveController = new PIDController(0.010, 0, 0.002, 0);

  Drivetrain m_drive;
  /** Creates a new AutoLevel. */
  public AutoLevel(Drivetrain drivetrain) {
    addRequirements(drivetrain);

    m_drive = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController.setTarget(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = Autonomous.navxGyro.getPitch();

    driveController.setInput(pitch);

    m_drive.drive(0, 0, driveController.calculate(1, -1));
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
