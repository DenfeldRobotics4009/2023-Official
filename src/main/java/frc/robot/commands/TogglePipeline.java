// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightServer;

public class TogglePipeline extends CommandBase {
  int pipeLine = 0; // When 0, be in cube mode hmm cube cube cube

  /** Creates a new TogglePipeline. */
  public TogglePipeline() {

    //m_limeLight = limeLight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // toggle from 0 to 1 and vice versa
    // Checking for 0 is faster on most CPUs
    if (pipeLine == 0) {pipeLine = 1;
    } else {pipeLine = 0;}

    LimelightServer.Pipeline.setNumber(pipeLine);
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
