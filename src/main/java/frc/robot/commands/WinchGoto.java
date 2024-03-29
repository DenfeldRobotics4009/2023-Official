// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class WinchGoto extends CommandBase {
  Arm m_arm;
  DoubleSupplier m_rotations;
  /** Creates a new WristGoto. */
  public WinchGoto(Arm arm, DoubleSupplier rotations) {
    addRequirements(arm);

    m_arm = arm;

    m_rotations = rotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.mC_winch.idealPosition = m_rotations.getAsDouble();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
