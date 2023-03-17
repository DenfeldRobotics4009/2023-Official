// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveKinematics extends SubsystemBase {

  private ShuffleboardTab kKinematicsTab = Shuffleboard.getTab("DriveKinematics");

  Translation2d robotPosition = new Translation2d();

  private GenericEntry 
    xPosEntry = kKinematicsTab.add("XPos", 0).getEntry(),
    yPosEntry = kKinematicsTab.add("YPos", 0).getEntry(),
    xSpeedEntry = kKinematicsTab.add("XSpeed", 0).getEntry(),
    ySpeedEntry = kKinematicsTab.add("YSpeed", 0).getEntry();

  /** Creates a new DriveKinimatics. */
  public DriveKinematics() {}

  /**
   * Updates the current robotPosition via gyro speeds
   */
  public void updatePosition(double intervalSeconds) {
    AHRS gyro = Autonomous.navxGyro;

    double angleRad = Math.toRadians(gyro.getAngle());

    // Grab current vilocities in meters per second
    Translation2d deltaV = new Translation2d(
      // Gyro X and Y values are robot relative
      gyro.getVelocityX()*Math.cos(angleRad) + gyro.getVelocityY()*Math.sin(angleRad), 
      gyro.getVelocityX()*Math.sin(angleRad) + gyro.getVelocityY()*Math.cos(angleRad)
    );

    robotPosition = robotPosition.plus(
      deltaV.times(intervalSeconds) // Multiply by seconds to calculate meters
    );
  }

  @Override
  public void periodic() {

    // Assume periodic is running at the default period
    updatePosition(Constants.kDefaultPeriod);

    xPosEntry.setDouble(robotPosition.getX());
    yPosEntry.setDouble(robotPosition.getY());

    xSpeedEntry.setDouble(Autonomous.navxGyro.getVelocityX());
    ySpeedEntry.setDouble(Autonomous.navxGyro.getVelocityY());
    
  }
}
