// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.NetworkTableEntryGroup;

import static frc.robot.Constants.*;

// OwO <- thats an owo not an uwu - Luke

public class Drivetrain extends SubsystemBase {

  private ShuffleboardTab kSwerveTab = Shuffleboard.getTab("SwerveControl");
  NetworkTableEntryGroup EncoderOffsetSet = new NetworkTableEntryGroup(kSwerveTab, "EOffset", Constants.MagOffset);
  NetworkTableEntryGroup SteerEncoderVal = new NetworkTableEntryGroup(kSwerveTab, "Enc", 0);
  NetworkTableEntryGroup DriveError = new NetworkTableEntryGroup(kSwerveTab, "DError", 0);

  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
  );

  // TODO Const
  //final double p = 0.001, i = 0, d = 0, f = 0, max = 0.8, min = -0.8;
  final double p = 0.005, i = 0, d = 0, f = 0;
  double max = 0.4, min = -0.4;
  final double adp = 25, adi = 0, add = 10, adf = 0, admax = 0.05, admin = -0.05;
  final double[] mctrlr_mFactor = {-1,1,1,-1};
  final double[] mTctrlr_mFactor = {1,1,1,1};
  final double[] mTctrlr_mAdditive = {0,0,0,0};

  // Var
  double[] recursiveError = {0.0,0.0,0.0,0.0};
  double jsZ, jsX, jsY; 

  // Creating object arrays, ik its a mess but expand things out as needed!
  final frc.robot.libraries.PIDController[] a_pid = {new frc.robot.libraries.PIDController(p, i, d, f), new frc.robot.libraries.PIDController(p, i, d, f),new frc.robot.libraries.PIDController(p, i, d, f), new frc.robot.libraries.PIDController(p, i, d, f)};
  final frc.robot.libraries.PIDController[] a_dpid = {new frc.robot.libraries.PIDController(adp, adi, add, adf), new frc.robot.libraries.PIDController(adp, adi, add, adf),new frc.robot.libraries.PIDController(adp, adi, add, adf), new frc.robot.libraries.PIDController(adp, adi, add, adf)};
  final CANSparkMax[] a_mctrlr = {new CANSparkMax(L1DM_, MotorType.kBrushless), new CANSparkMax(R1DM_, MotorType.kBrushless),new CANSparkMax(L2DM_, MotorType.kBrushless), new CANSparkMax(R2DM_, MotorType.kBrushless)};
  final CANSparkMax[] a_mTctrlr = {new CANSparkMax(L1SM_, MotorType.kBrushless),  new CANSparkMax(R1SM_, MotorType.kBrushless), new CANSparkMax(L2SM_, MotorType.kBrushless),  new CANSparkMax(R2SM_, MotorType.kBrushless)};
  public final RelativeEncoder[] a_mTencoder = {a_mTctrlr[0].getEncoder(), a_mTctrlr[1].getEncoder(),a_mTctrlr[2].getEncoder(), a_mTctrlr[3].getEncoder()};
  public final CANCoder[] a_mCANTencoder = {new CANCoder(Constants.L1SE_), new CANCoder(Constants.R1SE_), new CANCoder(Constants.L2SE_), new CANCoder(Constants.R2SE_)};
  public final RelativeEncoder[] a_mencoder = {a_mctrlr[0].getEncoder(), a_mctrlr[1].getEncoder(),a_mctrlr[2].getEncoder(), a_mctrlr[3].getEncoder()};

  /**
   * Creates a new drivetrain object for 4 MK4I swerve modules consisting of 8 NEO motors. -- 
   * Motor ids and constants are defined in Constants.java -- 
   * 
   * drive(X,Y,Z) updates joystick values, --
   * while motor speeds are updated periodically.
   * This is done to allow for simpler autonomous movement and easier override
   * for other commands. -- 
   * 
   * Each array is alligned in the format:
   *  { l1 , r1 , l2 , r2}
   */
  public Drivetrain() {

    for (int i = 0; i < 4; i++) {
      a_mTencoder[i].setPosition(0);
      a_mCANTencoder[i].configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
      a_mCANTencoder[i].configSensorDirection(true);
      a_dpid[i].setTolerance(20); // (RPM) arbitrary!!! #FIXME
      a_pid[i].setTolerance(6); // (Degreees)
      a_pid[i].setTarget(0);
    }
  }

  /**
   * Updates subsystem drive kinematics information.
   * Should be ran periodcally. Otherwise robot will
   * continue moving in last direction
   * @param jsz Joystick rotation input
   * @param jsx Joystick left/right input
   * @param jsy Joystick forward/backward input
   */
  public void drive(double jsz, double jsx, double jsy) {
    jsZ = jsz; jsX = jsx; jsY = jsy;
  }

  /**
   * Updates subsystem drive kinematics information.
   * Should be ran periodcally. Otherwise robot will
   * continue moving in last direction
   * 
   * This function converts the feild oriented controls into relative drive
   * 
   * TODO Test
   * 
   * @param jsz
   * @param jsx
   * @param jsy
   */
  public void feildOrientedDrive(double jsz, double jsx, double jsy) {
    double gyroAngle = Autonomous.navxGyro.getAngle();
    // (xcos(theta) - ysin(theta) , ycos(theta) + xsin(theta)) // Relative point of rad theta
    drive(
      jsz, // Black magic math
      jsX * Math.cos(gyroAngle) - jsY * Math.sin(gyroAngle),
      jsY * Math.cos(gyroAngle) + jsX * Math.sin(gyroAngle)
    );
  }

  void p_drive(SwerveModuleState... states) {
    for (int i = 0; i < states.length; i++) {
      // double encReadResult = (
      //   ((a_mTencoder[i].getPosition() * 360 * mTctrlr_mFactor[i] + mTctrlr_mAdditive[i]) 
      // ) / Constants.MK4I_STEER_RATIO) % (360); // 360 % 360 = 0
      double encReadResult = (
        (a_mCANTencoder[i].getAbsolutePosition() * mTctrlr_mFactor[i] + mTctrlr_mAdditive[i]) 
      ) % (360); // 360 % 360 = 0

      SteerEncoderVal.Entries[i].setDouble(encReadResult);

      double statesTurnResult = ((states[i].angle.getDegrees()) % (360));
      double distCorrection = calcDistCorrection(statesTurnResult, encReadResult);
      double[] optimizedCorrection = calcReverseCase(distCorrection);
    
      a_pid[i].setInput(optimizedCorrection[0]);
      double pidResult =  -a_pid[i].calculate(max, min);

      double tmps = states[i].speedMetersPerSecond;

      if (Math.abs(optimizedCorrection[0]) > Constants.DRIVE_TOLERANCE) {tmps = 0;}

      double errorOffset = calculateErrorOffset(
        i, (tmps / MAX_VELOCITY_METERS_PER_SECOND)
          * mctrlr_mFactor[i] * optimizedCorrection[1]
      );

      a_mTctrlr[i].set(pidResult);
      a_mctrlr[i].setVoltage(errorOffset);
    }
  }

  double calculateErrorOffset(int i, double S) {
    a_dpid[i].setTarget(0);

    double setSpeed = S * MAX_VOLTAGE;
    double idealRPM = S * MAX_NEO_RPM;
    double E = a_mencoder[i].getVelocity() - idealRPM;

    DriveError.Entries[i].setDouble(E);

    a_dpid[i].setInput(E);

    recursiveError[i] += a_dpid[i].calculate(admax, admin); 
    recursiveError[i] = clamp(-MAX_VOLTAGE, MAX_VOLTAGE, recursiveError[i]);
    return clamp(-MAX_VOLTAGE, MAX_VOLTAGE, setSpeed + recursiveError[i] );
  }

  double clamp(double in, double max, double min) {
    if (in > max) {return max;}
    if (in < min) {return min;}
    return in;
  }
  
  double ScaleToBattery(double input) {
    return clamp(input * (RobotController.getBatteryVoltage() - 12), input, 0.2);
  }

  @Override
  public void periodic() {
    // Read shuffleboard magent offset values and update CANCoders.
    for (int i = 0; i < 4; i++) {a_mCANTencoder[i].configMagnetOffset(
      EncoderOffsetSet.Entries[i].getDouble(Constants.MagOffset[i])
    );}
    // Scale max turning PID speed to battery value
    max = ScaleToBattery(1);
    min = -max;
    // Display values
    SmartDashboard.putNumber("Max Turning Power", max);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    // Pull wheel values from kinematics object
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(jsX, jsY, jsZ));
    // Drive based on pulled states
    p_drive(states);
  }

  /**
   * Set PIDController goal to zero
   * @param target PID goal
   * @param pos PID input
   * @return target relative to pos on a circle
   */
  double calcDistCorrection(double target, double pos) {
    if (Math.abs(target + (360) - pos) < Math.abs(target - pos)) {
      return target + (360) - pos;
    } else if (Math.abs(target - (360) - pos) < Math.abs(target - pos)) {
      return target - (360) - pos;
    } else {return target - pos;}
  }

  /**
   * @param distCorrectedTarget
   * @return {<New Target>, <Drive Factor (-1 or 1)}
   */
  double[] calcReverseCase(double distCorrectedTarget) {
    double[] out = {0,1};
    if (distCorrectedTarget > 90) {
      out[0] = distCorrectedTarget - 180; out[1] = -1;
    } else if (distCorrectedTarget < -90) {
      out[0] = distCorrectedTarget + 180; out[1] = -1;
    } else {out[0] = distCorrectedTarget;}
    return out;
  }
} // Class