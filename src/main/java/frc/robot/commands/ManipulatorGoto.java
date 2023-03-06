// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ManipulatorGoto extends CommandBase {
  Arm m_arm;
  Intake m_intake;
  DoubleSupplier m_armGoto, m_winchGoto, m_wristGoto;
  /** Creates a new ManipulatorGoto. */
  public ManipulatorGoto(
    Arm arm,
    DoubleSupplier armGoto, 
    DoubleSupplier winchGoto, 
    DoubleSupplier wristGoto
  ) {
    addRequirements(arm);
    m_arm = arm;
    m_armGoto = armGoto;
    m_winchGoto = winchGoto;
    m_wristGoto = wristGoto;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.mC_arm.idealPosition = m_armGoto.getAsDouble();
    m_arm.mC_winch.idealPosition = m_winchGoto.getAsDouble();

    if (Math.abs(m_arm.mC_arm.m_Encoder.getPosition() - m_arm.mC_arm.idealPosition) < 15) {
      m_arm.mC_wrist.idealPosition = m_wristGoto.getAsDouble();
    }
    m_arm.drive(0, 0, 0, false);
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
