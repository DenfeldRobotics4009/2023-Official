// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PovDrive extends CommandBase {
  Drivetrain m_Drivetrain;
  double m_speed;
  IntSupplier m_Pov;

  /** Creates a new PovDrive. */
  public PovDrive(Drivetrain DriveTrain, double speed, IntSupplier Pov) {
    addRequirements(DriveTrain);
    m_Drivetrain = DriveTrain;
    m_speed = speed;
    m_Pov = Pov;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("POV", m_Pov.getAsInt());
    switch (m_Pov.getAsInt()) {
      case 0:
        m_Drivetrain.drive(0, 0, -m_speed);
        break;
      case 45:
        m_Drivetrain.drive(0, m_speed, -m_speed);
        break;
      case 90:
        m_Drivetrain.drive(0, m_speed, 0);
        break;
      case 135:
        m_Drivetrain.drive(0, m_speed, m_speed);
        break;
      case 180:
        m_Drivetrain.drive(0, 0, m_speed);
        break;
      case 225:
        m_Drivetrain.drive(0, -m_speed, m_speed);
        break;
      case 270:
        m_Drivetrain.drive(0, -m_speed, 0);
        break;
      case 315:
        m_Drivetrain.drive(0, -m_speed, -m_speed);
        break;
      default:
        break;
    }
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
