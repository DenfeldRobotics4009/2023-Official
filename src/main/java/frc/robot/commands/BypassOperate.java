// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.subsystems.Arm;

public class BypassOperate extends CommandBase {
  Arm m_manipulator;
  DoubleSupplier m_arm, m_winch, m_wrist;
  /** Creates a new DefaultPitch. */
  public BypassOperate(
    Arm manipulator,

    DoubleSupplier arm,
    DoubleSupplier winch,
    DoubleSupplier wrist
  ) {
    m_arm = arm;
    m_winch = winch;
    m_wrist = wrist;

    m_manipulator = manipulator;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulator.drive(
      DeadZoneTuner.adjustForDeadzone(m_arm.getAsDouble() * 0.4, 0.16, false),
      m_winch.getAsDouble() * 0.2,
      m_wrist.getAsDouble() * 0.08,
      true
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
