// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// glory to felix the helix

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ApriltagServer;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
  //private final ApriltagServer m_aprilTagServer = new ApriltagServer();
  private final Autonomous m_autoSubsystem = new Autonomous(m_drivetrainSubsystem);

  private final Joystick m_jsDriver = new Joystick(0);
  private final Joystick m_jsOperator = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_jsDriver.getX() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_jsDriver.getY() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_jsDriver.getZ() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton d1 = new JoystickButton(m_jsDriver, 0);
    JoystickButton d2 = new JoystickButton(m_jsDriver, 2);
    JoystickButton d3 = new JoystickButton(m_jsDriver, 3);
    JoystickButton d4 = new JoystickButton(m_jsDriver, 4);
    JoystickButton d5 = new JoystickButton(m_jsDriver, 5);
    JoystickButton d6 = new JoystickButton(m_jsDriver, 6);
    JoystickButton d7 = new JoystickButton(m_jsDriver, 7);
    JoystickButton d8 = new JoystickButton(m_jsDriver, 8);
    JoystickButton d9 = new JoystickButton(m_jsDriver, 9);
    JoystickButton d10 = new JoystickButton(m_jsDriver, 10);
    JoystickButton d11 = new JoystickButton(m_jsDriver, 11);
    JoystickButton d12 = new JoystickButton(m_jsDriver, 12);

    JoystickButton o1 = new JoystickButton(m_jsOperator, 1);
    JoystickButton o2 = new JoystickButton(m_jsOperator, 2);
    JoystickButton o3 = new JoystickButton(m_jsOperator, 3);
    JoystickButton o4 = new JoystickButton(m_jsOperator, 4);
    JoystickButton o5 = new JoystickButton(m_jsOperator, 5);
    JoystickButton o6 = new JoystickButton(m_jsOperator, 6);
    JoystickButton o8 = new JoystickButton(m_jsOperator, 8);
    JoystickButton o7 = new JoystickButton(m_jsOperator, 7);
    JoystickButton o9 = new JoystickButton(m_jsOperator, 9);
    JoystickButton o11 = new JoystickButton(m_jsOperator, 11);
    JoystickButton o12 = new JoystickButton(m_jsOperator, 12);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
