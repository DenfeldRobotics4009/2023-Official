// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.subsystems.Manipulator;

public class DefaultOperate extends CommandBase {
  Manipulator m_manipulator;
  DoubleSupplier m_jsY, m_throttle;
  BooleanSupplier m_wristUp, m_wristDown, m_intake, m_outtake, m_override;
  IntSupplier m_pov;
  /** Creates a new DefaultPitch. */
  public DefaultOperate(
    Manipulator manipulator,
    DoubleSupplier jsY,
    DoubleSupplier Throttle,
    
    BooleanSupplier WristUp,
    BooleanSupplier WristDown,

    BooleanSupplier Intake,
    BooleanSupplier Outtake,

    BooleanSupplier Override,

    IntSupplier POV
  ) {
    m_jsY = jsY;
    m_throttle = Throttle;
    m_pov = POV;

    m_wristUp = WristUp;
    m_wristDown = WristDown;

    m_intake = Intake;
    m_outtake = Outtake;

    m_override = Override;

    m_manipulator = manipulator;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double scaler = (m_throttle.getAsDouble() + 1) / 2;

    // Interpret buttons
    double x = 0;
    // Pov returns -1 if not being pushed
    if (m_pov.getAsInt() == 0) {x = 0.15;
    } else if (m_pov.getAsInt() == 180) {x = -0.15;}

    double z = 0;
    if (m_wristUp.getAsBoolean()) {z = 0.06;
    } else if (m_wristDown.getAsBoolean()) {z = -0.06;}

    double in = 0;
    if (m_intake.getAsBoolean()) {in = 1;
    } else if (m_outtake.getAsBoolean()) {in = -1;}

    m_manipulator.drive(
      DeadZoneTuner.adjustForDeadzone(
        m_jsY.getAsDouble(), 0.1, false
      ) * scaler * 0.4,
      x * scaler,
      z * scaler,
      m_override.getAsBoolean()
    );

    m_manipulator.intake(in);

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
