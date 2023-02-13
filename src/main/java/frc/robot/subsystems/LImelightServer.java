// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightServer extends SubsystemBase {

  public static NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");

  public static NetworkTableEntry 
    tx = limeLight.getEntry("tx"), // Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
    ty = limeLight.getEntry("ty"), // Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
    tv = limeLight.getEntry("tv"), // Whether the limelight has any valid targets (0 or 1)
    /**
     * ledMode	Sets limelight’s LED state
     * 0	use the LED Mode set in the current pipeline
     * 1	force off
     * 2	force blink
     * 3	force on
     */
    LedMode = limeLight.getEntry("ledMode"),
    /**
     * camMode	Sets limelight’s operation mode
     * 0	Vision processor
     * 1	Driver Camera (Increases exposure, disables vision processing)
     */
    CamMode = limeLight.getEntry("camMode"),

    /**
     * Default pipeline is 0
     * 0 - Cube
     * 1 - Cone
     */
    Pipeline = limeLight.getEntry("pipeline");
    
  /** 
   * Creates a new LimelightServer. 
   * 
   * Regardless of the public static status of items in this class,
   * this subsystem will still need to be initialized for the code running
   * in this constructor.
   * 
   * Yes I'm aware its a single line :3
   */
  public LimelightServer() {
    LedMode.setNumber(1); // force lights off
    // Use defaults configured in the limelight webpage
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
