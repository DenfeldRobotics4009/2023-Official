// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveKinematics extends SubsystemBase {

  CANCoder[] turnEncoder;
  RelativeEncoder[] driveEncoder;

  double[] lastDistance = new double[4];

  private ShuffleboardTab kKinematicsTab = Shuffleboard.getTab("DriveKinematics");

  Pose2d robotPosition = new Pose2d();

  SwerveDriveOdometry odometry;

  private GenericEntry 
    xPosEntry = kKinematicsTab.add("XPos", 0).getEntry(),
    yPosEntry = kKinematicsTab.add("YPos", 0).getEntry(),
    xSpeedEntry = kKinematicsTab.add("XSpeed", 0).getEntry(),
    ySpeedEntry = kKinematicsTab.add("YSpeed", 0).getEntry();

  /** Creates a new DriveKinimatics. */
  public DriveKinematics(Drivetrain drivetrain) {

    // Grab encoder instances of both motors
    turnEncoder = drivetrain.a_mCANTencoder;
    driveEncoder = drivetrain.a_mencoder;

    odometry = new SwerveDriveOdometry(
      drivetrain.m_kinematics, 
      Autonomous.navxGyro.getRotation2d(), 
      new SwerveModulePosition[]{
        new SwerveModulePosition(driveEncoder[0].getPosition(), new Rotation2d(turnEncoder[0].getPosition())),
        new SwerveModulePosition(driveEncoder[1].getPosition(), new Rotation2d(turnEncoder[1].getPosition())),
        new SwerveModulePosition(driveEncoder[2].getPosition(), new Rotation2d(turnEncoder[2].getPosition())),
        new SwerveModulePosition(driveEncoder[3].getPosition(), new Rotation2d(turnEncoder[3].getPosition()))
      }
      // , new pos2d(0, 0, new Rotation2d(0)) // initial offset
    );
  }

  /**
   * Updates the current robotPosition via gyro speeds
   */
  public void updatePosition() {

    robotPosition = odometry.update(
      Autonomous.navxGyro.getRotation2d(), 
      new SwerveModulePosition[]{
        new SwerveModulePosition(driveEncoder[0].getPosition(), new Rotation2d(turnEncoder[0].getPosition())),
        new SwerveModulePosition(driveEncoder[1].getPosition(), new Rotation2d(turnEncoder[1].getPosition())),
        new SwerveModulePosition(driveEncoder[2].getPosition(), new Rotation2d(turnEncoder[2].getPosition())),
        new SwerveModulePosition(driveEncoder[3].getPosition(), new Rotation2d(turnEncoder[3].getPosition()))
      }
    );

    // Translation2d driveVectorAverage = new Translation2d();

    // double[] velocities = new double[4];

    // // Pair drive encoders with turn encoders
    // for (int i = 0; i < driveEncoder.length; i++) {

    //   velocities[i] = driveEncoder[i].getVelocity();
      
    //   // Calculate vector sum of all drive modules
    //   driveVectorAverage = driveVectorAverage.plus(
    //     new Translation2d(
    //       // Calculate distance difference from last frame
    //       driveEncoder[i].getPosition() - lastDistance[i], 
    //       // CANCoder is already configured to account for offset!
    //       new Rotation2d(Math.toRadians(turnEncoder[i].getPosition())) //+ Constants.MagOffset[i]
    //     )
    //   );
    // }

    // SmartDashboard.putNumber("L1Speed", velocities[0]);
    // SmartDashboard.putNumber("R1Speed", velocities[1]);
    // SmartDashboard.putNumber("L2Speed", velocities[2]);
    // SmartDashboard.putNumber("R2Speed", velocities[3]);

    SmartDashboard.putNumber("L1Direction", turnEncoder[0].getAbsolutePosition());
    SmartDashboard.putNumber("R1Direction", turnEncoder[1].getAbsolutePosition());
    SmartDashboard.putNumber("L2Direction", turnEncoder[2].getAbsolutePosition());
    SmartDashboard.putNumber("R2Direction", turnEncoder[3].getAbsolutePosition());

    // // Divide by 4 for average
    // driveVectorAverage = driveVectorAverage.div(driveEncoder.length);

    AHRS gyro = Autonomous.navxGyro;

    //double angleRad = Math.toRadians(gyro.getAngle());

    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    // robotPosition = robotPosition.plus(
    //   driveVectorAverage.rotateBy(
    //     new Rotation2d(angleRad)
    //   )
    // );

    // // Record last distance for delta calculation
    // for (int i = 0; i < 4; i ++) {
    //   lastDistance[i] = driveEncoder[i].getPosition(); 
    // }
  }

  @Override
  public void periodic() {

    // Assume periodic is running at the default period
    updatePosition();

    xPosEntry.setDouble(robotPosition.getX());
    yPosEntry.setDouble(robotPosition.getY());

    // xSpeedEntry.setDouble(Autonomous.navxGyro.getVelocityX());
    // ySpeedEntry.setDouble(Autonomous.navxGyro.getVelocityY());
    
  }
}
