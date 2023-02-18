// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.PIDController;

/**
 * Among us lore easter egg!!!1!11!!!!1!
 */
public class Autonomous extends SubsystemBase {
  Drivetrain m_driveTrain;

  double meanDriveRotations, m_thetaDegreesTurnDelta;

  // TODO Tune!
  PIDController 
    driveController = new PIDController(0.1, 0, 0, 0), 
    turnController = new PIDController(0.1, 0, 0, 0);

  public static AHRS navxGyro = new AHRS();

  /** 
   * Creates a new Autonomous. 
  */
  public Autonomous(Drivetrain m_DrivetrainSubsystem) {
    m_driveTrain = m_DrivetrainSubsystem;

    turnController.setTarget(0);
  }

  /**
   * Updates meanDriveRotations via sum of quarter of each drive motor
   */
  void updateMeanDriveRotations() {
    meanDriveRotations = 0;
    for (int i = 0; i < 4; i++) {
      meanDriveRotations += m_driveTrain.a_mencoder[i].getPosition() / 4;
    }

  }

  /**
   * Sets the goal values for the turn function
   * 
   * @param thetaDegreesDelta degrees to turn relative to current direction
   * @param speed The speed (from -1 to 1) to travel
   * 
   * @returns self for chaining
   */ 
  public Autonomous setTurnGoal(double thetaDegreesDelta) {
    m_thetaDegreesTurnDelta = thetaDegreesDelta;

    return this;
  }

  /**
   * @param distanceTolerance
   * @param speed
   * @return jsZ value to turn
   */
  double turn(double speed) {

    turnController.setInput(
      calcDistCorrection(m_thetaDegreesTurnDelta, navxGyro.getAngle())
    );

    return turnController.calculate(speed, -speed);
  }

  /**
   * Turns the robot towards the target based upon the given error
   * @param Error Distance of the target from the center of the robot
   * @param pidTuner libraries.PIDController pidtuner object tuned for this function.
   * setTarget should be set to 0 prior to calling this function.
   * @param maxTurnSpeed Maximum outputs of the PID function
   */
  public void turn(double Error, PIDController pidTuner, double maxTurnSpeed, double jsx, double jsy) {
    pidTuner.setInput(Error);

    // SmartDashboard.putNumber("Target Error", Error);

    m_driveTrain.drive(
      pidTuner.calculate(maxTurnSpeed, -maxTurnSpeed), 
      jsx, 
      jsy
    );
  }

  /**
   * Set PIDController goal to zero
   * @param target PID goal
   * @param pos PID input
   * @return target relative to pos on a circle
   */
  public double calcDistCorrection(double target, double pos) {
    if (Math.abs(target + (360) - pos) < Math.abs(target - pos)) {
      return target + (360) - pos;
    } else if (Math.abs(target - (360) - pos) < Math.abs(target - pos)) {
      return target - (360) - pos;
    } else {return target - pos;}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
