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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultOperate;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.LoadingStationDistanceDrive;
import frc.robot.commands.OuttakeCone;
import frc.robot.commands.OuttakeCube;
import frc.robot.commands.PovDrive;
import frc.robot.commands.TogglePipeline;
import frc.robot.commands.WristGoto;
import frc.robot.commands.AimTowardsTarget;
import frc.robot.commands.ArmGoto;
import frc.robot.commands.BypassOperate;
import frc.robot.commands.Calibrate;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightServer;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrainSubsystem = new Drivetrain();

  private final Arm m_manipulator = new Arm();
  private final Intake m_intake = new Intake();

  private final Autonomous m_autoSubsystem = new Autonomous(m_drivetrainSubsystem);

  private final LimelightServer m_limeLight = new LimelightServer();

  private final Joystick m_jsDriver = new Joystick(0);
  private final Joystick m_jsOperator = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDrive(
            m_drivetrainSubsystem,
            () -> m_jsDriver.getTrigger(),
            () -> m_jsDriver.getX() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_jsDriver.getY() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_jsDriver.getZ() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    
    m_manipulator.setDefaultCommand(
      new DefaultOperate(
        m_manipulator, 
        () -> m_jsOperator.getY(),
        () -> m_jsOperator.getThrottle(),

        () -> m_jsOperator.getRawButton(7), // Wrist Up
        () -> m_jsOperator.getRawButton(9), // Wrist Down

        () -> m_jsOperator.getPOV()
      )
    );

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

    // JSY -- Swerve Drive
    // JSX --
    // JSZ -- 
    // HAT 1 -- HAT drive
    // HAT 2
    // HAT 3
    // HAT 4
    // HAT 5
    // HAT 6
    // HAT 7
    // HAT 8
    // JoystickButton d1 = new JoystickButton(m_jsDriver, 1); // Precision mode
    JoystickButton d2 = new JoystickButton(m_jsDriver, 2); // Auto lineup
    JoystickButton d3 = new JoystickButton(m_jsDriver, 3); // Switch pipeline
    JoystickButton d4 = new JoystickButton(m_jsDriver, 4);
    JoystickButton d5 = new JoystickButton(m_jsDriver, 5);
    JoystickButton d6 = new JoystickButton(m_jsDriver, 6);
    JoystickButton d7 = new JoystickButton(m_jsDriver, 7);
    JoystickButton d8 = new JoystickButton(m_jsDriver, 8);
    JoystickButton d9 = new JoystickButton(m_jsDriver, 9);
    JoystickButton d10 = new JoystickButton(m_jsDriver, 10);
    JoystickButton d11 = new JoystickButton(m_jsDriver, 11);
    JoystickButton d12 = new JoystickButton(m_jsDriver, 12);
    
    POVButton dpov0 = new POVButton(m_jsDriver, 0);
    POVButton dpov45 = new POVButton(m_jsDriver, 45);
    POVButton dpov90 = new POVButton(m_jsDriver, 90);
    POVButton dpov135 = new POVButton(m_jsDriver, 135);
    POVButton dpov180 = new POVButton(m_jsDriver, 180);
    POVButton dpov225 = new POVButton(m_jsDriver, 225);
    POVButton dpov270 = new POVButton(m_jsDriver, 270);
    POVButton dpov315 = new POVButton(m_jsDriver, 315);

    double speed = 0.01;
    dpov0.or(dpov45.or(dpov90.or(dpov135.or(dpov180.or(dpov225.or(dpov270.or(dpov315))))))).whileTrue(
      new PovDrive(m_drivetrainSubsystem, speed, () -> m_jsDriver.getPOV())
    );
    
    d3.onTrue(new TogglePipeline());
    d2.whileTrue(
      new AimTowardsTarget(
        m_autoSubsystem, m_drivetrainSubsystem, 
        () -> m_jsDriver.getX() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> m_jsDriver.getY() * Constants.MAX_VELOCITY_METERS_PER_SECOND
      )
    );

    d5.whileTrue(
      new LoadingStationDistanceDrive(
          m_drivetrainSubsystem,
          () -> m_jsDriver.getTrigger(),
          () -> m_jsDriver.getX() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
          () -> m_jsDriver.getY() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
          () -> m_jsDriver.getZ() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        )
    );

    JoystickButton o1 = new JoystickButton(m_jsOperator, 1);
    JoystickButton o2 = new JoystickButton(m_jsOperator, 2);
    JoystickButton o3 = new JoystickButton(m_jsOperator, 3);
    JoystickButton o4 = new JoystickButton(m_jsOperator, 4);
    JoystickButton o5 = new JoystickButton(m_jsOperator, 5);
    JoystickButton o6 = new JoystickButton(m_jsOperator, 6); 
    JoystickButton o8 = new JoystickButton(m_jsOperator, 8);
    //JoystickButton o7 = new JoystickButton(m_jsOperator, 7); // wrist down
    //JoystickButton o9 = new JoystickButton(m_jsOperator, 9); // Wrist up
    JoystickButton o10 = new JoystickButton(m_jsOperator, 10);
    JoystickButton o11 = new JoystickButton(m_jsOperator, 11);
    JoystickButton o12 = new JoystickButton(m_jsOperator, 12);

    /**
     * .and(o1.negate()) Makes the command require the trigger to not be
     * pressed to function. Thus, the trigger can act as a function key
     */
    o11.and(o12).and(o1.negate()).onFalse(new Calibrate(m_manipulator));
    o11.and(o12).and(o1.negate()).whileTrue(
      new BypassOperate(
        m_manipulator, 
        () -> m_jsOperator.getY(),
        () -> m_jsOperator.getThrottle(),

        () -> m_jsOperator.getRawButton(7), // Wrist Up
        () -> m_jsOperator.getRawButton(9), // Wrist Down

        () -> m_jsOperator.getPOV()
      )
    );

    o8.and(o1.negate()).whileTrue(new WristGoto(m_manipulator, () -> Constants.WristCubeIntakeRot));
    o10.and(o1.negate()).whileTrue(new WristGoto(m_manipulator, () -> Constants.WristConeFlipRot));
    o2.and(o1.negate()).whileTrue(new WristGoto(m_manipulator, () -> -1));

    o8.and(o1).whileTrue(new ArmGoto(m_manipulator, () -> Constants.ConePlace2Rot));
    o10.and(o1).whileTrue(new ArmGoto(m_manipulator, () -> Constants.CubePlace2Rot));
    o12.and(o1).whileTrue(new ArmGoto(m_manipulator, () -> Constants.ConePlace1Rot));

    o5.whileTrue(new IntakeCube(m_intake));
    o3.whileTrue(new OuttakeCube(m_intake));

    o6.whileTrue(new IntakeCone(m_intake));
    o4.whileTrue(new OuttakeCone(m_intake));
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
