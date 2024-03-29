// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double SWERVEWHEEL_CIRCUMFERANCE = Math.PI*0.09401432; // 3.9

    public static final double ENCODER_GEAR_RATIO = 1 / 8.14;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.65405;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     */
    public static final double MAX_VOLTAGE = 12.0;

    /**
     * FIXME  Measure the drivetrain's maximum velocity or calculate the theoretical
     * 
     * The maximum velocity of the robot in meters per second. 
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.6576;//3.6576; // Theoretical
    
    public static final double ROTATIONS_TO_METERS = 0.03793762588 * 2; // r * this = meters

            // 100.0 / 60.0 *
            // SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            // SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

    // // For auto, maximum meters per second input to drive motors
    // public static final double AUTO_MAXIMUM_METERS_PER_SECOND = 0.059;

    /**
     * Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double MK4I_STEER_RATIO = 150/7;

    /**
     * The maxmimum angle the current wheel angle can be from the target 
     * to allow drive motors to engage.
     */
    public static final double DRIVE_TOLERANCE = 80; // Degrees

    public static final int MAX_NEO_RPM = 5676;

    public static final int L1DM_ = 21; 
    public static final int L1SM_ = 31;
    public static final int L1SE_ = 0;

    public static final int R1DM_ = 23; 
    public static final int R1SM_ = 33; 
    public static final int R1SE_ = 2;

    public static final int L2DM_ = 22; 
    public static final int L2SM_ = 32; 
    public static final int L2SE_ = 1;

    public static final int R2DM_ = 24; 
    public static final int R2SM_ = 34; 
    public static final int R2SE_ = 3;

    public static final int PneumaticHubID = 60;

    public static final double L1SMagnetOffset = -5;
    public static final double R1SMagnetOffset = -120;
    public static final double L2SMagnetOffset = 109.5;
    public static final double R2SMagnetOffset = -59;

    public static final int JointM_ = 0;

    public static final double[] MagOffset = {
        L1SMagnetOffset, R1SMagnetOffset, L2SMagnetOffset, R2SMagnetOffset
    };

    public static final int ArmID = 41; 
    public static final int WinchID = 40; 
    public static final int WristID = 42; 
    public static final int IntakeID = 50; 

    public static final double ArmMaximumLengthRot = 114;
    public static final double ArmMaximumHightRot = -90; // Use in min
    public static final double WristMaximumAngleRot = -60; // use in min

    public static final double WristLoadingStationIntake = -50.5;
    public static final double ArmLoadingStationIntake = -72.0;

    public static final double LoadingStationDistGoal = 45.5;

    public static final double intakeSpeed = 1; // 0.3 default, harass tanner

    public static final double LimelightDegreesTarget = -15;

    public static final double WristCubeIntakeRot = -34;
    public static final double WristConeFlipRot = -36;

    public static final double ConePlace1Rot = -49.66;
    public static final double ConePlace2Rot = -81.80;
    public static final double CubePlace2Rot = -57.40;

    public static final double WristConePickup = -30.71;

    // public static final int DistSensorID = 0;

    public static final double kDefaultPeriod = 0.02; // {edu.wpi.first.wpilibj.TimedRobot}
}
