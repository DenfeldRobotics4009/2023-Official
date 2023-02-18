// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Calibrate extends CommandBase {
  Arm m_arm;
  /** Creates a new Calibrate. */
  public Calibrate(Arm armSubsystem) {
    addRequirements(armSubsystem);

    m_arm = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.mC_arm.m_Encoder.setPosition(0);
    m_arm.mC_arm.idealPosition = 0;

    m_arm.mC_winch.m_Encoder.setPosition(0);
    m_arm.mC_winch.idealPosition = 0;

    m_arm.mC_wrist.m_Encoder.setPosition(0);
    m_arm.mC_wrist.idealPosition = 0;
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
