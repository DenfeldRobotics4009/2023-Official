// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.subsystems.Arm;

public class BypassOperate extends CommandBase {
  Arm m_manipulator;
  DoubleSupplier m_jsY, m_throttle, m_jsS;
  BooleanSupplier m_wristUp, m_wristDown, m_intake, m_outtake, m_override1, m_override2;
  IntSupplier m_pov;
  /** Creates a new DefaultPitch. */
  public BypassOperate(
    Arm manipulator,

    DoubleSupplier jsY,
    DoubleSupplier Throttle,
    DoubleSupplier jsSlider,

    IntSupplier POV
  ) {
    m_jsY = jsY;
    m_throttle = Throttle;
    m_pov = POV;

    m_jsS = jsSlider;

    m_manipulator = manipulator;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double scaler = (m_throttle.getAsDouble() + 1);

    // Interpret buttons
    double x = 0;
    // Pov returns -1 if not being pushed
    if (m_pov.getAsInt() == 0) {x = 0.15;
    } else if (m_pov.getAsInt() == 180) {x = -0.15;}

    m_manipulator.drive(
      DeadZoneTuner.adjustForDeadzone(
        m_jsY.getAsDouble(), 0.1, false
      ) * scaler * 0.4,
      x * scaler,
      m_jsS.getAsDouble() * 0.08 * scaler,
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
