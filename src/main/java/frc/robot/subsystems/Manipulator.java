// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

  /**
   * Currently the manipulator end of our robot is
   * undecided whether it will be actuaded by pneumatics or
   * a motor. Each is a different design.
   */

  SpeedControlledCANSparkMax m_arm = new SpeedControlledCANSparkMax(
    new CANSparkMax(Constants.ArmID, MotorType.kBrushless)
  );

  /** Creates a new Manipulator. */
  public Manipulator() {}

  @Override
  public void periodic() {
    // TODO Tune please!!!!!!!!!!!!
    m_arm.setPID(
      0, 0, 0
    ).setMaximums(
      0, 0
    );
  }

  /**
   * @param speed The speed of the arm motor
   */
  public void yControl(double speed) {
    m_arm.set(speed);
  }

  /**
   * Run this function periodically when yControl is not being ran,
   * thus keeping m_arm reacting to gravity or other things.
   */
  public void noControl() {
    m_arm.set();
  }
}
