// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.AutoCommand.EventMarkerCommandMode.CommandScheduleMode;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Drivetrain;

public class AutoCommand extends CommandBase {

  public static class EventMarkerCommandMode {
    /**
     * InnerAutoCommand
     */
    public interface CommandScheduleMode {
      /**
       * @param input Whether the run condition is met
       * @return true if the command should run
       */
      public boolean poll(boolean input);
    }

    public static class OnTrue implements CommandScheduleMode {

      boolean trip = false;

      /**
       * Returns true once when input is true.
       * Will not return true again until input is false
       */
      public boolean poll(boolean input) {
        boolean prev = trip;
        trip = input;
        return !prev && input;
      }
    }

    public static class OnFalse implements CommandScheduleMode {

      boolean trip = false;
      /**
       * Returns true once when input is false.
       * Will not return true again until input is true
       */
      public boolean poll(boolean input) {
        boolean prev = trip;
        trip = !input;
        return !prev && !input;
      }
    }

    public static class WhileTrue implements CommandScheduleMode {

      /**
       * Returns true while input is true
       */
      public boolean poll(boolean input) {
        return input;
      }
    }
  }

  /**
   * Pairs a pathPlanner event marker with a 
   * command, and run information
   * 
   * This class functions as a struct
   */
  public static class EventMarkerSet{
    /**
     * Pathplanner marker, holds command run time
     */
    public EventMarker kEventMarker;
    /**
     * Command to run
     */
    public Command kCommand;
    /**
     * Method to run the command
     */
    public CommandScheduleMode kOperationMode;
    /**
     * How long to poll the command
     */
    public double kWindowSeconds;
    /**
     * How long to delay path planner path
     */
    public double kDelaySeconds;
    /**
     * Binds a pathplanner marker, and command together.
     * 
     * @param eventMarker Pathplanner marker
     * 
     * @param command WPILib command
     * 
     * @param commandMode AutoCommand.EventMarkerCommandMode
     * 
     * @param operationWindowSeconds Time to delay path planner, and run command
     */
    public EventMarkerSet(
      EventMarker eventMarker, 
      Command command,
      CommandScheduleMode commandMode,
      double operationWindowSeconds,
      double delayWindowSeconds
    ) {
      kEventMarker = eventMarker;

      kCommand = command;

      kOperationMode = commandMode;

      kWindowSeconds = operationWindowSeconds;

      kDelaySeconds = delayWindowSeconds;
    }
    /**
     * Binds a pathplanner marker, and command together.
     */
    public EventMarkerSet() {}
  }

  public static ShuffleboardTab m_tab = Shuffleboard.getTab("Auto");

  public static GenericEntry 
    Vilocity = m_tab.addPersistent(
      "aVilocity", 0
    ).getEntry(), 
    AngularVilocity = m_tab.addPersistent(
      "aAngularVilocity", 0
    ).getEntry(), 
    Time = m_tab.addPersistent(
      "aTime", 0
    ).getEntry(), 
    EndTime  = m_tab.addPersistent(
      "aEndTime", 0
    ).getEntry();

  PIDController distanceCorrectionController = new PIDController(0.1, 0, 0, 0);

  Timer m_altTimer = new Timer(), m_pathTimer = new Timer();

  double timerOffset = 0;

  Drivetrain m_drive;

  PathPlannerTrajectory m_path;

  EventMarkerSet[] m_EventMarkerSetArr;

  double initialDistanceSample = 0;

  /**
   * 
   * @param DriveSubsystem
   * @param PathPlannerPath
   * @param EventMarkerOperations
   */
  public AutoCommand(
    Drivetrain DriveSubsystem, 
    PathPlannerTrajectory PathPlannerPath,
    EventMarkerSet... EventMarkerOperations
  ) {
    addRequirements(DriveSubsystem);

    m_drive = DriveSubsystem;

    m_path = PathPlannerPath;

    m_EventMarkerSetArr = EventMarkerOperations;

    // Error based pid
    distanceCorrectionController.setTarget(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pathTimer.start();

    // Sample encoders on startup
    initialDistanceSample = average(
      m_drive.a_mencoder[0].getPosition(),
      m_drive.a_mencoder[1].getPosition(),
      m_drive.a_mencoder[2].getPosition(),
      m_drive.a_mencoder[3].getPosition()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Grab state at current time, offset by timerOffset
    // Cast due to PathPlannerState functioning as Trajectory.State
    PathPlannerState m_state = (PathPlannerState) m_path.sample(m_pathTimer.get());
    // Extract speeds from state, thus allowing them to be set to 0
    double MetersPerSecond = m_state.velocityMetersPerSecond;
    double RadPerSecond = m_state.angularVelocityRadPerSec;
    //Parse through marker set
    for (EventMarkerSet markerSet : m_EventMarkerSetArr) {
      /**
       * Alternate time records how long
       * path time has been paused
       */
      boolean Event = false;
      // Check to schedule command
      if (
        // Poll via user defined method via interface
        markerSet.kOperationMode.poll(
          m_pathTimer.get() >= markerSet.kEventMarker.timeSeconds
          && m_altTimer.get() < markerSet.kWindowSeconds
        )
      ) {
        /**
         * Schedule command if the time has passed marker schedule time,
         * and time is under the window limit.
         */
        markerSet.kCommand.schedule();
      }

      // Check to pause path
      if (
        m_pathTimer.get() >= markerSet.kEventMarker.timeSeconds
        && m_altTimer.get() < markerSet.kDelaySeconds
      ) {
        Event = true;

        MetersPerSecond = 0;
        RadPerSecond = 0;

        // pause
        m_altTimer.start();
        m_pathTimer.stop();
      }
      // Repeat check for all markers

      /**
       * If no event is active, reset alt timer
       */
      if (!Event) {
        m_altTimer.reset();
        m_altTimer.stop();
      }
    } // end for loop

    // Extract states, invert, and drive.
    // Inverting inputs makes the robot drive forward
    m_drive.drive(
      -RadPerSecond, 
      // This may be false, if it is, use m_state.holonomicRotation funny buiseness
      0, // Zero, as PathPlanner always drives in the direction the robot is facing
      -MetersPerSecond
    );

    //m_drive.periodic();


    // Display values

    Vilocity.setDouble(MetersPerSecond);

    AngularVilocity.setDouble(RadPerSecond);

    Time.setDouble(m_pathTimer.get());
    
    EndTime.setDouble(m_path.getTotalTimeSeconds());

    /**
     * Display travelled distance, the final number should
     * equal to the path length specified on path planner
     */

    double travelDist = average(
      m_drive.a_mencoder[0].getPosition(),
      m_drive.a_mencoder[1].getPosition(),
      m_drive.a_mencoder[2].getPosition(),
      m_drive.a_mencoder[3].getPosition()
    ) - initialDistanceSample;

    SmartDashboard.putNumber(
      "Travelled Auto Distance", travelDist / Constants.ROTATIONS_TO_METERS
    );
    
  }

  double average(double... in) {
    double out = 0;
    for (double d : in) {out += d;}
    return out / in.length;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pathTimer.get() >= m_path.getTotalTimeSeconds();
  }
}
