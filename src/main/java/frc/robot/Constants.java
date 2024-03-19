// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

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

    public static final int MAX_SPEED = 9000;
    public static final int SLOW_SPEED = 5000;
    public static final int HYPER_SPEED = 11000;
    // public static final int TRUE_MAX_SPEED = 13000; // actual max speed it is capable of???

    // OFFSET values changed on 1/14/23 to fix widebot conumdrum
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0.185 - 0.25;
    public static final boolean FRONT_LEFT_REVERSE_DRIVE = true;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0.44 - 0.25; 
    public static final boolean FRONT_RIGHT_REVERSE_DRIVE = true;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0.27 + 0.25; 
    public static final boolean BACK_LEFT_REVERSE_DRIVE = true;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR =  17; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0.9 + 0.25;
    public static final boolean BACK_RIGHT_REVERSE_DRIVE = true;

    public static final double STEER_MOTOR_GEAR_RATIO = 12.8 / 1;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14 / 1; // could potentially be 6.75:1 depending on if it is L1 of L2
                                                                  // see https://www.swervedrivespecialties.com/products/mk4-swerve-module?variant=39376675012721
    public static final double WHEEL_DIAMETER = 0.1016; // 4 inches

    public static final double ROTATION_KP = 0.012;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0.001;

    public static final double TRANSLATION_KP = 1.0;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0.01;

    // CAMERA AND TARGET CONSTANTS
    public static final double CAMERA_HEIGHT_METERS = 0.6223;  // TODO: set to actual value
    public static final double CAMERA_PITCH_RADIANS = Math.toRadians(22);
    public static final double SHOOTER_HINGE_ELEVATION_METERS = 0.127;
    public static final double SHOOTER_CAMERA_OFFSET_METERS = 0.2032;

    public static final double SPEAKER_TARGET_HEIGHT_METERS = 1.368; // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/Apriltag_Images_and_User_Guide.pdf
    public static final double SPEAKER_OPENING_BOTTOM = 1.985; // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf
    public static final double SPEAKER_OPENING_TOP = 2.105;
    public static final double SPEAKER_GOAL_X_OFFSET = 0.23;
    public static final double SPEAKER_OPENING_HEIGHT_METERS = (SPEAKER_OPENING_TOP + SPEAKER_OPENING_BOTTOM) / 2;
    public static final double SPEAKER_TIPPY_TOP_METERS = 2.489;
    public static final int[] SPEAKER_RIGHT_IDS = {3, 7};
    public static final int[] SPEAKER_CENTER_IDS = {4, 8};
    
    public static final int[] AMP_IDS = {5, 6};

    // INTAKE CONSTANTS
    public static final int TOP_INTAKE_MOTOR_ID = 15;
    public static final int BOTTOM_INTAKE_MOTOR_ID = 24;
    public static final double INTAKE_SPEED = 0.5;
    public static final double REVERSE_INTAKE_SPEED = -0.2;

    // SHOOTER CONSTANTS
    public static final int SHOOT_MOTOR_ID = 5;

    public static final double SHOOTER_KP = 0.0007;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.00001;

    public static final double SHOOT_MARGIN_OF_ERROR = 50.0;
    public static final double SHOOTER_GEAR_RATIO = 18 / 14;
    public static final double SHOOTER_WHEEL_DIAMETER = 0.054;
    public static final double SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_DIAMETER * Math.PI;
    public static final double TYPICAL_SPEED = 11;

    // FEEDER CONSTANTS
    public static final int FEEDER_MOTOR_ID = 27;
    public static final double FEEDER_SPEED = 0.5;
    public static final double FEEDER_REVERSE_SPEED = -0.2;

    // SCREW ACTUATOR CONSTANTS
    public static final int SCREW_MOTOR_ID = 21;
    public static final int HINGE_ENCODER_ID = 0;

    public static final double SCREW_KP = 0.0048;
    public static final double SCREW_KI = 0.0025;
    public static final double SCREW_KD = 0.00001;

    public static final double HINGE_KP = 70.0;
    public static final double HINGE_KI = 0.00;
    public static final double HINGE_KD = 0.1;
    public static final double HINGE_MARGIN_OF_ERROR = 0.002;

    public static final double SCREW_GEAR_RATIO = 5/1;
    public static final double SCREW_ROTATIONS_PER_INCH = 10;
    public static final double MAX_SCREW_INCHES = 6.25;
    public static final double MAX_SCREW_ROTATIONS = MAX_SCREW_INCHES * SCREW_ROTATIONS_PER_INCH;
    public static final double HINGE_MAX_ANGLE = 0.585; // 0.89
    public static final double HINGE_MIN_ANGLE = 0.468; // 0.795
    public static final double SCREW_SPEED = 0.75;

    public static final double EXTENSION_BEFORE_SCREW = 8.25;
    public static final double SCREW_HINGE_DROPDOWN = 1.9;
    public static final double SHOOTER_LENGTH_TO_SCREW_HINGE = 14;
    public static final double SCREW_MOTOR_HEIGHT = 7.5;
    public static final double SHOOTER_HINGE_X = 4;

    public static final double SPEAKER_SETPOINT = 0.526;
    public static final double AMP_SETPOINT = 0.524;
    public static final double SIDE_SETPOINT = 0.527;
  
    public static final double HINGE_SHOOTER_M = -434.053;
    public static final double HINGE_SHOOTER_B = 290.634;

    // CLIMBER CONSTANTS
    public static final int LEFT_ARM_ID = 10;
    public static final int RIGHT_ARM_ID = 23;

    public static final double PULLEY_DIAMETER = 0.01905;
    public static final double PULLEY_RADIUS = PULLEY_DIAMETER / 2;
    public static final double PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI;
    public static final double CLIMBER_GEAR_RATIO = 45/1;

    public static final double CLIMBER_SPEED = 0.5;
  }
