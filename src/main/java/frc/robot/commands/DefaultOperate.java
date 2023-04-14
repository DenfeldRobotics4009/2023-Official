// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.subsystems.Arm;

public class DefaultOperate extends CommandBase {
  Arm m_manipulator;
  DoubleSupplier m_arm, m_winch, m_wrist, m_pov;
  /** Creates a new DefaultPitch. */
  public DefaultOperate(
    Arm manipulator,

    DoubleSupplier arm,
    DoubleSupplier winch,
    DoubleSupplier wrist,

    DoubleSupplier pov
  ) {
    m_arm = arm;
    m_winch = winch;
    m_wrist = wrist;

    m_pov = pov;

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
      false
    );

    SmartDashboard.putNumber("AAAAPOV", m_pov.getAsDouble());

    double armSpeed = 0;
    if (m_pov.getAsDouble() != -1) {
      if (m_pov.getAsDouble() == 0) { // Nested if statements dont judge me please this is bad
        armSpeed = 0.1;
      } else if (m_pov.getAsDouble() == 180) {
        armSpeed = -0.1;
      }
    }

    m_manipulator.drive(
      -armSpeed * 0.4, 0, 0, false
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
