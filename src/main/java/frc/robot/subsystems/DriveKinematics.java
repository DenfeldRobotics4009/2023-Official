// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.swervedrivespecialties.swervelib.SwerveModule;

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

  double[] initialDriveEncoderValue = new double[4];

  double[] lastDistance = new double[4];

  private ShuffleboardTab kKinematicsTab = Shuffleboard.getTab("DriveKinematics");

  public SwerveDriveOdometry odometry;

  private GenericEntry 
    driveEncoderEntry = kKinematicsTab.add("DriveEncoderVal", 0).getEntry(),
    xPosEntry = kKinematicsTab.add("XPos", 0).getEntry(),
    yPosEntry = kKinematicsTab.add("YPos", 0).getEntry(),
    rotation0Entry = kKinematicsTab.add("Rotation0", 0).getEntry(),
    rotation1Entry = kKinematicsTab.add("Rotation1", 0).getEntry();

  /** Creates a new DriveKinimatics. */
  public DriveKinematics(Drivetrain drivetrain) {

    // Grab encoder instances of both motors
    turnEncoder = drivetrain.a_mCANTencoder;
    driveEncoder = drivetrain.a_mencoder;

    odometry = new SwerveDriveOdometry(
      drivetrain.m_kinematics, 
      Autonomous.navxGyro.getRotation2d(), 
      getZeroPosition()
    );

    resetOdometry();

    xPosEntry.setDouble(odometry.getPoseMeters().getX());
    yPosEntry.setDouble(odometry.getPoseMeters().getY());

    
  }

  /**
   * Updates the current robotPosition via gyro speeds
   */
  public void updatePosition() {

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      swerveModulePositions[i] = new SwerveModulePosition(
        // Calculate meters from drive rotations
        (driveEncoder[i].getPosition() - initialDriveEncoderValue[i])* (Constants.SWERVEWHEEL_CIRCUMFERANCE * Constants.ENCODER_GEAR_RATIO),
        // Rotation2d works in radians
        new Rotation2d(Math.toRadians(turnEncoder[i].getAbsolutePosition()))
      );
    }

    odometry.update(
      Autonomous.navxGyro.getRotation2d(), 
      new SwerveModulePosition[] {
        new SwerveModulePosition(
          // Calculate meters from drive rotations
          (driveEncoder[0].getPosition() - initialDriveEncoderValue[0])* (Constants.SWERVEWHEEL_CIRCUMFERANCE * Constants.ENCODER_GEAR_RATIO),
          // Rotation2d works in radians
          new Rotation2d(Math.toRadians(turnEncoder[0].getAbsolutePosition()))
        ),
        new SwerveModulePosition(
          // Calculate meters from drive rotations
          (driveEncoder[1].getPosition() - initialDriveEncoderValue[1])* (Constants.SWERVEWHEEL_CIRCUMFERANCE * Constants.ENCODER_GEAR_RATIO),
          // Rotation2d works in radians
          new Rotation2d(Math.toRadians(turnEncoder[1].getAbsolutePosition()))
        ),
        new SwerveModulePosition(
          // Calculate meters from drive rotations
          (driveEncoder[2].getPosition() - initialDriveEncoderValue[2])* (Constants.SWERVEWHEEL_CIRCUMFERANCE * Constants.ENCODER_GEAR_RATIO),
          // Rotation2d works in radians
          new Rotation2d(Math.toRadians(turnEncoder[2].getAbsolutePosition()))
        ),
        new SwerveModulePosition(
          // Calculate meters from drive rotations
          (driveEncoder[3].getPosition() - initialDriveEncoderValue[3])* (Constants.SWERVEWHEEL_CIRCUMFERANCE * Constants.ENCODER_GEAR_RATIO),
          // Rotation2d works in radians
          new Rotation2d(Math.toRadians(turnEncoder[3].getAbsolutePosition()))
        )
      }
    );

    SmartDashboard.putNumber("L1Direction", Math.toRadians(turnEncoder[0].getAbsolutePosition()));
    SmartDashboard.putNumber("R1Direction", Math.toRadians(turnEncoder[1].getAbsolutePosition()));
    SmartDashboard.putNumber("L2Direction", Math.toRadians(turnEncoder[2].getAbsolutePosition()));
    SmartDashboard.putNumber("R2Direction", Math.toRadians(turnEncoder[3].getAbsolutePosition()));

    AHRS gyro = Autonomous.navxGyro;

    SmartDashboard.putNumber("Gyro", gyro.getAngle());

  }

  public void resetOdometry() {
    odometry.resetPosition(
      new Rotation2d(Autonomous.navxGyro.getAngle()), 
      getZeroPosition(), 
      new Pose2d(0, 0, new Rotation2d())
    );

    for (int i = 0; i < initialDriveEncoderValue.length; i++) {
      initialDriveEncoderValue[i] = driveEncoder[i].getPosition();
    }
  }

  public SwerveModulePosition[] getZeroPosition() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d(Math.toRadians(turnEncoder[0].getPosition() - Constants.MagOffset[0]))),
      new SwerveModulePosition(0, new Rotation2d(Math.toRadians(turnEncoder[1].getPosition() - Constants.MagOffset[1]))),
      new SwerveModulePosition(0, new Rotation2d(Math.toRadians(turnEncoder[2].getPosition() - Constants.MagOffset[2]))),
      new SwerveModulePosition(0, new Rotation2d(Math.toRadians(turnEncoder[3].getPosition() - Constants.MagOffset[3])))
    };
  }

  @Override
  public void periodic() {

    // Assume periodic is running at the default period
    updatePosition();

    driveEncoderEntry.setDouble(driveEncoder[0].getPosition() * (Constants.SWERVEWHEEL_CIRCUMFERANCE * Constants.ENCODER_GEAR_RATIO));

    rotation0Entry.setDouble(turnEncoder[0].getAbsolutePosition() % 360);
    rotation1Entry.setDouble(turnEncoder[1].getAbsolutePosition() % 360);

    xPosEntry.setDouble(odometry.getPoseMeters().getX()); //* 39.37 * 5.267881531);
    yPosEntry.setDouble(odometry.getPoseMeters().getY()); //* 39.37 * 5.267881531);

    // xSpeedEntry.setDouble(Autonomous.navxGyro.getVelocityX());
    // ySpeedEntry.setDouble(Autonomous.navxGyro.getVelocityY());
    
  }
}
