// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  PneumaticHub m_hub = new PneumaticHub(Constants.PneumaticHubID);

  Solenoid 
    led_yellow = m_hub.makeSolenoid(0),
    led_purple = m_hub.makeSolenoid(1);

  /** Creates a new LED. */
  public LED() {
    led_yellow.set(false);
    led_purple.set(false);
  }

  public void requestCube() {
    led_purple.set(true);
    led_yellow.set(false);
  }

  public void requestCone() {
    led_purple.set(false);
    led_yellow.set(true);
  }

  public void off() {
    led_purple.set(false);
    led_yellow.set(false);
  }

  @Override
  public void periodic() {}
}
