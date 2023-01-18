// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Struct;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.NetworkTableEntryGroup;

public class ApriltagServer extends SubsystemBase {
  // Create the cameraServer tab
  private ShuffleboardTab kCameraTab = Shuffleboard.getTab("CameraServer");
  NetworkTableEntryGroup cornersX, cornersY;
  // Microsoft_LifeCam_HD-3000 is the default name for the camera
  PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  PhotonPipelineResult latestResult; // Periodically update the latest result
  PhotonTrackedTarget latestTarget;  // and latest target, latestTarget will be null
                                     // if there is none
  /** Creates a new ApriltagServer. */
  public ApriltagServer() {
    cornersX = new NetworkTableEntryGroup(kCameraTab, "cornerX", 0);
    cornersY = new NetworkTableEntryGroup(kCameraTab, "cornerY", 0);
  }

  /**
   * @return PhotonTrackedTarget
   */
  PhotonTrackedTarget getBestTarget() {
    return latestResult.getBestTarget();
  }

  /**
   * This function requires the camera to be at a fixed angle and position
   * @param target PhotonTrackedTarget target
   * @returns the distance of the target from the camera via image hight in inches
   */
  public double getDistance(
    // theta = sadness
    PhotonTrackedTarget target, double cameraHight, 
    double targetHight, double cameraPitchDegrees
  ) {
    // Calculate range via photon utilities
    return PhotonUtils.calculateDistanceToTargetMeters(
      cameraHight,targetHight,Math.toRadians(cameraPitchDegrees),
      Units.degreesToRadians(target.getPitch()));
  }

  @Override
  public void periodic() {
    latestResult = photonCamera.getLatestResult();
    latestTarget = getBestTarget();
    // For debugging reasons
    SmartDashboard.putBoolean("Valid Target(S)", latestResult.hasTargets());
    if (latestResult.hasTargets()) {
      SmartDashboard.putNumber("Distance", getDistance(
        getBestTarget(), 12, 50, 25
      ));
    } else {SmartDashboard.putNumber("Distance", -1);}

  }
}
