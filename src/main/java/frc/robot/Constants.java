// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.SqlTimeSerializer;

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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;  // 25 in
    public static final double DRIVETRAIN_TRACKLENGTH_METERS = 0.508;  // 20 in
    public static final double DRIVE_THETA_OFFSET = Math.toDegrees(Math.atan(DRIVETRAIN_TRACKLENGTH_METERS/DRIVETRAIN_TRACKWIDTH_METERS));
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    // public static final double DRIVETRAIN_WHEELBASE_METERS = 0.48895;

    public static final int MAX_SPEED = 1000;

    // OFFSET values changed on 1/14/23 to fix widebot conumdrum
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 169;
    public static final boolean FRONT_LEFT_REVERSE_DRIVE = true;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 159.4; 
    public static final boolean FRONT_RIGHT_REVERSE_DRIVE = true;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 159.0; 
    public static final boolean BACK_LEFT_REVERSE_DRIVE = true;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 144;
    public static final boolean BACK_RIGHT_REVERSE_DRIVE = false;

    public static final double STEER_MOTOR_GEAR_RATIO = 12.8 / 1;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14 / 1; // could potentially be 6.75:1 depending on if it is L1 of L2
                                                                  // see https://www.swervedrivespecialties.com/products/mk4-swerve-module?variant=39376675012721
    public static final double WHEEL_DIAMETER = 0.1016; // 4 inches


    // INTAKE CONSTANTS
    public static final int LEFT_INTAKE_MOTOR_ID = 9;
    public static final int RIGHT_INTAKE_MOTOR_ID = 10;
    public static final double INTAKE_SPEED = 0.3;

    // SHOOTER CONSTANTS
    public static final int FEEDER_MOTOR_ID = 11;
    public static final int SCREW_DRIVE_MOTOR_ID = 12;
    public static final int LEFT_SHOOT_MOTOR_ID = 13;
    public static final int RIGHT_SHOOT_MOTOR_ID = 14;

    public static final double SHOOTER_KP = 0.00007;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;

    public static final double SCREW_KP = 0.0;
    public static final double SCREW_KI = 0.0;
    public static final double SCREW_KD = 0.0;

    public static final double FEEDER_SPEED = 0.3;
    public static final double SCREW_DRIVE_GEAR_RATIO = 1.0;
    public static final double MAX_SCREW_DRIVE_ROTATIONS = 50;
  }
