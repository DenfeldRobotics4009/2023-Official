// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.PositionControlledCANSparkMax;


public class Arm extends SubsystemBase {

  public static ShuffleboardTab kArmTab = Shuffleboard.getTab("Manipulator");

  GenericEntry winchEncoderEntry, armEncoderEntry, wristEncoderEntry;

  CANSparkMax m_arm, m_winch, m_wrist;

  public static Rev2mDistanceSensor distSensor;

  public PositionControlledCANSparkMax mC_wrist, mC_winch, mC_arm;

  /** Creates a new Manipulator. */
  public Arm() {

    distSensor = new Rev2mDistanceSensor(Port.kOnboard);

    distSensor.setAutomaticMode(true);

    // TODO Tune pid of the three musketeers, and set them to position control
    m_winch = new CANSparkMax(Constants.WinchID, MotorType.kBrushless);
    mC_winch = new PositionControlledCANSparkMax(m_winch, "winch");

    m_arm = new CANSparkMax(Constants.ArmID, MotorType.kBrushless);
    mC_arm = new PositionControlledCANSparkMax(m_arm, "arm");

    m_wrist = new CANSparkMax(Constants.WristID, MotorType.kBrushless);
    mC_wrist = new PositionControlledCANSparkMax(m_wrist, "wrist");

    mC_arm.setPID(0.12, 0, 0.02);
    mC_wrist.setPID(0.08, 0, 0);

    winchEncoderEntry = kArmTab.add("winchEncoder Value", 0).getEntry();
    armEncoderEntry = kArmTab.add("armEncoder Value", 0).getEntry();
    wristEncoderEntry = kArmTab.add("wristEncoder Value", 0).getEntry();

    m_winch.getEncoder().setPosition(0);
    m_arm.getEncoder().setPosition(0);
    m_wrist.getEncoder().setPosition(0);
  }

  /**
   * Drives each component of the arm based upon the given inputs.
   * @param deltaX Arm
   * @param deltaY Winch
   * @param deltaZ Wrist
   * @param override True to ignore safety limits
   */
  public void drive(double deltaX, double deltaY, double deltaZ, boolean override) {

    if (override) {
      mC_winch.setMaximums(1, 0, 0);
      mC_arm.setMaximums(1, 0, 0);
      mC_wrist.setMaximums(1, 0, 0);
    } else {
      mC_winch.setMaximums(
        1, Constants.ArmMaximumLengthRot, 0.5
      );
      mC_arm.setMaximums(
        1, 1, Constants.ArmMaximumHightRot
      );
      mC_wrist.setMaximums(
        1, 1.5, Constants.WristMaximumAngleRot
      );
    }

    mC_winch.set(deltaY);
    mC_arm.set(deltaX);
    mC_wrist.set(deltaZ);
  
  }

  public void Reset() {
    
  }

  @Override
  public void periodic() {
    mC_wrist.Update();

    SmartDashboard.putNumber("Dist", distSensor.GetRange());
  }
}
