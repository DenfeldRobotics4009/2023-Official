// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.65405;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     */
    public static final double MAX_VOLTAGE = 12.0;

    /**
     * FIXME  Measure the drivetrain's maximum velocity or calculate the theoretical.
     * 
     * The maximum velocity of the robot in meters per second. BASE 5880.0
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 100.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
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
    public static final double DRIVE_TOLERANCE = 45; // Degrees

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

    public static final double L1SMagnetOffset = -75;
    public static final double R1SMagnetOffset = 5;
    public static final double L2SMagnetOffset = 85;
    public static final double R2SMagnetOffset = 230;

    public static final int JointM_ = 0;

    public static final double[] MagOffset = {
        L1SMagnetOffset, R1SMagnetOffset, L2SMagnetOffset, R2SMagnetOffset
    };

    public static final int ArmID = 41; // TODO
    public static final int WinchID = 40; // TODO
    public static final int WristID = 42; // TODO
    public static final int IntakeID = 50; // TODO

    public static final double ArmMaximumLengthRot = 200; // twoo hundo
    public static final double ArmMaximumHightRot = -75; // Use in min
    public static final double WristMaximumAngleRot = -30; // use in min

    public static final double intakeSpeed = 0.3;
}
