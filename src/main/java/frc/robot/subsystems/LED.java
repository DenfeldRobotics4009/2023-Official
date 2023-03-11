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
    led_yellow1 = m_hub.makeSolenoid(0), 
    led_yellow2 = m_hub.makeSolenoid(15), 

    led_purple1 = m_hub.makeSolenoid(14),
    led_purple2 = m_hub.makeSolenoid(2);

  /** Creates a new LED. */
  public LED() {
    led_yellow1.set(false);
    led_purple1.set(false);
    led_yellow2.set(false);
    led_purple2.set(false);
  }

  public void requestCube() {
    led_yellow1.set(false);
    led_purple1.set(true);
    led_yellow2.set(false);
    led_purple2.set(true);
  }

  public void requestCone() {
    led_yellow1.set(true);
    led_purple1.set(false);
    led_yellow2.set(true);
    led_purple2.set(false);
  }

  public void off() {
    led_yellow1.set(false);
    led_purple1.set(false);
    led_yellow2.set(false);
    led_purple2.set(false);
  }

  @Override
  public void periodic() {}
}
