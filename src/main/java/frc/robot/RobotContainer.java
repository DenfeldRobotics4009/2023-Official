// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

// glory to felix the helix

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultOperate;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.LoadingStationDistanceDrive;
import frc.robot.commands.ManipulatorGoto;
import frc.robot.commands.OuttakeCone;
import frc.robot.commands.OuttakeCube;
import frc.robot.commands.PovDrive;
import frc.robot.commands.RequestCone;
import frc.robot.commands.RequestCube;
import frc.robot.commands.Reset;
import frc.robot.commands.StopIntake;
import frc.robot.commands.TogglePipeline;
import frc.robot.commands.WinchGoto;
import frc.robot.commands.WristGoto;
import frc.robot.libraries.DeadZoneTuner;
import frc.robot.commands.AimTowardsTarget;
import frc.robot.commands.ArmGoto;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.BypassOperate;
import frc.robot.commands.Calibrate;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveKinematics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimelightServer;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<Integer> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrainSubsystem = new Drivetrain();

  private final Arm m_manipulator = new Arm();
  private final Intake m_intake = new Intake();

  private final Autonomous m_autoSubsystem = new Autonomous(m_drivetrainSubsystem);

  private final LimelightServer m_limeLight = new LimelightServer();

  private final DriveKinematics m_kinenatics = new DriveKinematics();
  
  private final LED m_led = new LED();

  private final Joystick m_jsDriver = new Joystick(0);
  private final Joystick m_jsOperator = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Initalize USB camera to display feed on shuffleboard
    CameraServer.startAutomaticCapture(0);

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDrive(
          m_drivetrainSubsystem,
          () -> m_jsDriver.getTrigger(),

          () -> m_jsDriver.getRawButton(8),

          () -> DeadZoneTuner.adjustForDeadzone(
            m_jsDriver.getX(), 0.15, false
          ) * Constants.MAX_VELOCITY_METERS_PER_SECOND,

          () -> DeadZoneTuner.adjustForDeadzone(
            m_jsDriver.getY(), 0.15, false
          ) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
          
          () -> DeadZoneTuner.adjustForDeadzone(
            m_jsDriver.getZ(), 0.16, false
          ) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    
    m_manipulator.setDefaultCommand(
      new DefaultOperate(
        m_manipulator, 
        () -> DeadZoneTuner.adjustForDeadzone(m_jsOperator.getRawAxis(2), 0.16, false),
        () -> m_jsOperator.getRawAxis(4),
        () -> m_jsOperator.getRawAxis(1)
      )
    );

    autoChooser.addOption("None", 0);
    autoChooser.setDefaultOption("SingleConeClimb A", 1);
    autoChooser.addOption("SingleCone A", 2);

    autoChooser.addOption("SingleCone B", 3);

    SmartDashboard.putData("Autonomous", autoChooser);
    
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
    //JoystickButton d7 = new JoystickButton(m_jsDriver, 7);
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

    double speed = 0.8;
    dpov0.or(dpov45.or(dpov90.or(dpov135.or(dpov180.or(dpov225.or(dpov270.or(dpov315))))))).whileTrue(
      new PovDrive(m_drivetrainSubsystem, speed, () -> m_jsDriver.getPOV())
    );

    d4.toggleOnTrue(new RequestCube(m_led));
    d6.toggleOnTrue(new RequestCone(m_led));
    
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

          () -> DeadZoneTuner.adjustForDeadzone(
            m_jsDriver.getX(), 0.15, false
          ) * Constants.MAX_VELOCITY_METERS_PER_SECOND,

          () -> DeadZoneTuner.adjustForDeadzone(
            m_jsDriver.getY(), 0.15, false
          ) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
          
          () -> DeadZoneTuner.adjustForDeadzone(
            m_jsDriver.getZ(), 0.15, false
          ) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        )
    );

    d11.whileTrue(new AutoLevel(m_drivetrainSubsystem));

    JoystickButton o1 = new JoystickButton(m_jsOperator, 1);
    JoystickButton o2 = new JoystickButton(m_jsOperator, 2);
    JoystickButton o3 = new JoystickButton(m_jsOperator, 3);
    JoystickButton o4 = new JoystickButton(m_jsOperator, 4);
    JoystickButton o5 = new JoystickButton(m_jsOperator, 5);
    JoystickButton o6 = new JoystickButton(m_jsOperator, 6); 
    JoystickButton o8 = new JoystickButton(m_jsOperator, 8);
    JoystickButton o7 = new JoystickButton(m_jsOperator, 7);
    JoystickButton o9 = new JoystickButton(m_jsOperator, 9);
    JoystickButton o10 = new JoystickButton(m_jsOperator, 10);
    JoystickButton o11 = new JoystickButton(m_jsOperator, 11);
    JoystickButton o12 = new JoystickButton(m_jsOperator, 12);

    /**
     * .and(o1.negate()) Makes the command require the trigger to not be
     * pressed to function. Thus, the trigger can act as a function key
     */
    o11.and(o1.negate()).onFalse(new Calibrate(m_manipulator));
    o11.and(o1.negate()).whileTrue(
      new BypassOperate(
        m_manipulator, 
        () -> m_jsOperator.getRawAxis(2),
        () -> m_jsOperator.getRawAxis(4),
        () -> m_jsOperator.getRawAxis(1)
      )
    );

    o8.onTrue(new Reset(m_manipulator));

    o5.and(o2.negate()).whileTrue(new WristGoto(m_manipulator, () -> Constants.WristCubeIntakeRot));
    o6.and(o2.negate()).whileTrue(new WristGoto(m_manipulator, () -> Constants.WristConeFlipRot));
    o7.and(o2.negate()).whileTrue(new WristGoto(m_manipulator, () -> -1));

    o9.onTrue(new SequentialCommandGroup(
      new WristGoto(m_manipulator, () -> Constants.WristLoadingStationIntake),
      new ArmGoto(m_manipulator, () -> Constants.ArmLoadingStationIntake)
    ));

    //o5.and(o2).whileTrue(new ArmGoto(m_manipulator, () -> Constants.ConePlace2Rot));
    o6.and(o2).whileTrue(new ArmGoto(m_manipulator, () -> Constants.CubePlace2Rot));
    o7.and(o2).whileTrue(new ArmGoto(m_manipulator, () -> Constants.ConePlace1Rot));

    // SequentialCommandGroup conePlace2 = new SequentialCommandGroup();
    // conePlace2.addCommands(
    //   new ArmGoto(m_manipulator, () -> Constants.ConePlace2Rot),
    //   new WinchGoto(m_manipulator, () -> 100)
    // );
    // conePlace2.andThen(new WaitCommand(0.5));
    // conePlace2.andThen(new WristGoto(m_manipulator, () -> -45));
    o5.and(o2).whileTrue(
      new ManipulatorGoto(m_manipulator, 
      () -> Constants.ConePlace2Rot, 
      () -> 100,
      () -> -43)
    );

    o1.and(o4.negate()).whileTrue(new IntakeCube(m_intake));
    o3.and(o4.negate()).whileTrue(new OuttakeCube(m_intake));

    o1.and(o4).whileTrue(new IntakeCone(m_intake));
    o3.and(o4).whileTrue(new OuttakeCone(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {    
    switch (autoChooser.getSelected()) {

      case 1:
        PathPlannerTrajectory m_path = PathPlanner.loadPath(
          "1ConeClimb1", 2.8, 2);

        return new AutoCommand(
          m_drivetrainSubsystem, 
          m_path,
          true,
          (DriverStation.getAlliance() == DriverStation.Alliance.Red) ? (true) : (false),
          new AutoCommand.EventMarkerSet(
            m_path.getMarkers().get(0), 
            new ManipulatorGoto(
              m_manipulator,
              () -> -67.64, 
              () -> 88.71, 
              () -> -38.49), 
            new AutoCommand.EventMarkerCommandMode.WhileTrue(), 
            2, 
            2),
          
          new AutoCommand.TimerScheduledCommand(
            new OuttakeCone(m_intake), 
            new AutoCommand.EventMarkerCommandMode.OnTrue(), 
            1, 
            1.5),
          
          new AutoCommand.TimerScheduledCommand(
            new StopIntake(m_intake), 
            new AutoCommand.EventMarkerCommandMode.OnTrue(), 
            1, 
            2.5),

          new AutoCommand.EventMarkerSet(
            m_path.getMarkers().get(1), 
            new Reset(m_manipulator),
            new AutoCommand.EventMarkerCommandMode.OnTrue(), 
            2.5, 
            0),

          new AutoCommand.EventMarkerSet(
            m_path.getMarkers().get(2), 
            new AutoLevel(m_drivetrainSubsystem), 
            new AutoCommand.EventMarkerCommandMode.WhileTrue(), 
            8, 
            8)
          // Marker 1
          // ,new AutoCommand.EventMarkerSet(
          //   m_path.getMarkers().get(0),                                                                                                                                                                                                                      
          //   new ArmGoto(m_manipulator, () -> Constants.ConePlace2Rot),
          //   new AutoCommand.EventMarkerCommandMode.OnTrue(),
          //   2,
          //   1
          // ),
          // new AutoCommand.EventMarkerSet(
          //   m_path.getMarkers().get(1), 
          //   new ArmGoto(m_manipulator, () -> 0),
          //   new AutoCommand.EventMarkerCommandMode.OnTrue(),
          //   2,
          //   1
          // )
        );

      case 2:
        PathPlannerTrajectory m_path2 = PathPlanner.loadPath(
          "1Cone1", 2.3, 2);

        return new AutoCommand(
          m_drivetrainSubsystem, 
          m_path2,
          true,
          (DriverStation.getAlliance() == DriverStation.Alliance.Red) ? (true) : (false),
          new AutoCommand.EventMarkerSet(
            m_path2.getMarkers().get(0), 
            new ManipulatorGoto(
              m_manipulator,
              () -> -67.64, 
              () -> 88.71, 
              () -> -38.49), 
            new AutoCommand.EventMarkerCommandMode.WhileTrue(), 
            2, 
            2),
          
            new AutoCommand.TimerScheduledCommand(
              new OuttakeCone(m_intake), 
              new AutoCommand.EventMarkerCommandMode.OnTrue(), 
              1, 
              1.5),
          
            new AutoCommand.TimerScheduledCommand(
              new StopIntake(m_intake), 
              new AutoCommand.EventMarkerCommandMode.OnTrue(), 
              1, 
              2.5),

          new AutoCommand.EventMarkerSet(
            m_path2.getMarkers().get(1), 
            new Reset(m_manipulator),
            new AutoCommand.EventMarkerCommandMode.OnTrue(), 
            2.5, 
            0)
        );

      case 3:

        PathPlannerTrajectory m_path3 = PathPlanner.loadPath(
          "1Cone2", 2.3, 2);
          

        return new AutoCommand(
          m_drivetrainSubsystem, 
          m_path3,
          true,
          (DriverStation.getAlliance() == DriverStation.Alliance.Red) ? (true) : (false),
          new AutoCommand.EventMarkerSet(
            m_path3.getMarkers().get(0), 
            new ManipulatorGoto(
              m_manipulator,
              () -> -67.64, 
              () -> 88.71, 
              () -> -38.49), 
            new AutoCommand.EventMarkerCommandMode.WhileTrue(), 
            2, 
            2),
          
            new AutoCommand.TimerScheduledCommand(
              new OuttakeCone(m_intake), 
              new AutoCommand.EventMarkerCommandMode.OnTrue(), 
              1, 
              1.5),
          
            new AutoCommand.TimerScheduledCommand(
              new StopIntake(m_intake), 
              new AutoCommand.EventMarkerCommandMode.OnTrue(), 
              1, 
              2.5),

          new AutoCommand.EventMarkerSet(
            m_path3.getMarkers().get(1), 
            new Reset(m_manipulator),
            new AutoCommand.EventMarkerCommandMode.OnTrue(), 
            2.5, 
            0)
        );

      default:
        return new InstantCommand(); // Does nothing
    }
  }
}
