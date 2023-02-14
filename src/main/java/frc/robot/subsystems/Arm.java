// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.PositionControlledCANSparkMax;

public class Arm extends SubsystemBase {

  public static ShuffleboardTab kArmTab = Shuffleboard.getTab("Manipulator");

  GenericEntry winchEncoderEntry, armEncoderEntry, wristEncoderEntry;

  CANSparkMax m_arm, m_winch, m_wrist;

  PositionControlledCANSparkMax mC_wrist, mC_winch, mC_arm;

  /** Creates a new Manipulator. */
  public Arm() {



    // TODO Tune pid of the three musketeers, and set them to position control
    m_winch = new CANSparkMax(Constants.WinchID, MotorType.kBrushless);
    mC_winch = new PositionControlledCANSparkMax(m_winch, "winch");

    m_arm = new CANSparkMax(Constants.ArmID, MotorType.kBrushless);
    mC_arm = new PositionControlledCANSparkMax(m_arm, "arm");

    m_wrist = new CANSparkMax(Constants.WristID, MotorType.kBrushless);
    mC_wrist = new PositionControlledCANSparkMax(m_wrist, "wrist");

    winchEncoderEntry = kArmTab.add("winchEncoder Value", 0).getEntry();
    armEncoderEntry = kArmTab.add("armEncoder Value", 0).getEntry();
    wristEncoderEntry = kArmTab.add("wristEncoder Value", 0).getEntry();

    m_winch.getEncoder().setPosition(0);
    m_arm.getEncoder().setPosition(0);
    m_wrist.getEncoder().setPosition(0);
  }

  /**
   * X limits are equal to the arm rotation limits
   */

  // /**
  //  * @param x
  //  * @return The minimum y value at the given x value
  //  */
  // double yMinimum(double x) {
  //   return Math.sqrt(
  //     Constants.ArmMinimumLengthRot*Constants.ArmMinimumLengthRot - x*x
  //   );
  // }

  // /**
  //  * @param x
  //  * @return The maximum y value at the given x value
  //  */
  // double yMaximum(double x) {
  //   return Math.sqrt(
  //     Constants.ArmMaximumLengthRot*Constants.ArmMinimumLengthRot - x*x
  //   );
  // }

  /**
   * Drives the arm to the given coordinates
   * @param deltaX Arm
   * @param deltaY Winch
   * @param Z Wrist offset from parallel
   */
  public void drive(double deltaX, double deltaY, double Z, boolean override) {

    // x = PositionControlledCANSparkMax.clamp(
    //   x + deltaX, Constants.ArmMaximumLengthRot, Constants.ArmMinimumLengthRot
    // );
    // y = PositionControlledCANSparkMax.clamp(
    //   y + deltaY, yMaximum(x), yMinimum(x)
    // );
    
    // // Z is not a delta, the input will be a range from 0 to 1, and correlates to the offset 
    // z = ((Math.PI - Constants.InitialWristAngleRad) * Z) / (Math.PI * 2);

    if (override) {
      mC_winch.setMaximums(
        1, 0, 0, false
      );
      mC_arm.setMaximums(
        1, 0, 0, false
      );
      mC_wrist.setMaximums(
        1, 0, 0, false
      );
    } else {
      mC_winch.setMaximums(
        1, Constants.ArmMaximumLengthRot, 0.5, false
      );
      mC_arm.setMaximums(
        1, 0.5, Constants.ArmMaximumHightRot, false
      );
      mC_wrist.setMaximums(
        1, 0.1, Constants.WristMaximumAngleRot, false
      );
    }

    mC_winch.set(deltaY);
    mC_arm.set(deltaX);
    mC_wrist.set(Z);
  
  }
  
  public void Reset() {
    
  }

  @Override
  public void periodic() {
    // /**
    //  * TODO It may be better to create a function within PositionControlledCANSparkMax.java that
    //  * sets the idealPosition variable and runs the set() function, just to let this code
    //  * make a tiny bit more sense
    //  */

    // // Calculate solutions for the three musketeers
    // mc_arm.idealPosition = ( // Rotations
    //   (Math.PI/2) - Constants.InitialArmAngleRad - Math.atan(y / x) // solve in radians
    // ) / (Math.PI * 2); // Divide to convert to rotations
    // mc_arm.set(); // Periodically set motors, because this is jank and cringe im going insnae weeoooweeoo

    // mc_winch.idealPosition = Math.sqrt(x*x + y*y); // Rotations via pythag
    // mc_winch.set(); 
    // winchEntry.setDouble(mc_winch.idealPosition);

    // mc_wrist.idealPosition = PositionControlledCANSparkMax.clamp(
    //   ( // Rotations
    //     Math.PI - Constants.InitialWristAngleRad - Math.atan(y / x) // solve in radians
    //   ) / (Math.PI * 2) - z  // Divide to convert to rotations, subtract the inputted offset
    //   // Clamp to limits, we are adding an offset so we have to clamp a second time!
    //   , (Math.PI - Constants.InitialWristAngleRad) / (Math.PI * 2)
    //   , Constants.InitialWristAngleRad / (Math.PI * 2)
    // );
    // mc_wrist.set();

    winchEncoderEntry.setDouble(m_winch.getEncoder().getPosition());
    armEncoderEntry.setDouble(m_arm.getEncoder().getPosition());
    wristEncoderEntry.setDouble(m_wrist.getEncoder().getPosition());
  }
}
