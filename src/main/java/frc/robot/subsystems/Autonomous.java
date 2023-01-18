// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.PIDController;

/**
 * Among us lore easter egg!!!1!11!!!!1!
 */
public class AutonomousSubsystem extends SubsystemBase {
  DrivetrainSubsystem m_driveTrain;
  // ApriltagServer m_cameraServer;
  /** Creates a new Autonomous. */
  public AutonomousSubsystem(
    DrivetrainSubsystem m_DrivetrainSubsystem
    // ,ApriltagServer m_ApriltagServer
  ) {
    m_driveTrain = m_DrivetrainSubsystem;
    // m_cameraServer = m_ApriltagServer;
  }

  double m_speed, m_thetaDegreesDelta, m_goalMeanDriveRotations, meanDriveRotations;
  PIDController driveController = new PIDController(0.1, 0, 0, 0);

  void updateMeanDriveRotations() {
    meanDriveRotations = 0;
    for (int i = 0; i < 4; i++) {
      meanDriveRotations += m_driveTrain.a_mencoder[i].getPosition() / 4;
    }
  }
  /**
   * Sets the goal values for the drive function.
   * 
   * @param thetaDegreesDelta The direction to travel in degrees from the current direction
   * @param driveRotationsDelta The number of motor rotations to travel
   * @param speed The speed (from -1 to 1) to travel
   */
  public void setDriveGoal(double thetaDegreesDelta, double driveRotationsDelta, double speed) {
    updateMeanDriveRotations();

    m_goalMeanDriveRotations = meanDriveRotations + driveRotationsDelta;

    m_thetaDegreesDelta = thetaDegreesDelta;
    m_speed = speed;
  }

  /**
   * Called periodically instead of DriveTrainSubsystem.drive to allow for 
   * autonomous control.
   * 
   * @param distanceToTolerance the tolerance of driveRotationsDelta
   * 
   * @returns true if traveled distance is within thew goal by the distance tolerance
   */
  public boolean drive(double distanceTolerance) {
    updateMeanDriveRotations();

    // Update PID controller values periodically
    driveController.setInput(meanDriveRotations);
    driveController.setTarget(m_goalMeanDriveRotations);

    // Scale trig function speeds according to pid outputs, set max to the defined speed.
    // This requires functional PID tuning!
    double pidSpeed = driveController.calculate(-m_speed, m_speed);

    // TODO Ignore jsZ
    /**
     * The issue with turning while were driving is that the jsX and jsY values
     * are based on our direction. This swerve bot is NOT feild oriented.
     */
    double emulated_jsX = pidSpeed*Math.cos(Math.toRadians(m_thetaDegreesDelta));
    double emulated_jsY = pidSpeed*Math.sin(Math.toRadians(m_thetaDegreesDelta));
    m_driveTrain.drive(0, emulated_jsX, emulated_jsY);

    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
