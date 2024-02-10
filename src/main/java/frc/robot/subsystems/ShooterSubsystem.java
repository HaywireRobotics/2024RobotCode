// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ShooterSubsystem extends SubsystemBase {
  private final NEO feederMotor;
  private final NEO screwDriveMotor;
  private final NEO leftShootMotor;
  private final NEO rightShootMotor;

  private double shootSpeed = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.feederMotor = new NEO(Constants.FEEDER_MOTOR_ID);
    this.screwDriveMotor = new NEO(Constants.SCREW_DRIVE_MOTOR_ID);
    this.leftShootMotor = new NEO(Constants.LEFT_SHOOT_MOTOR_ID);
    this.rightShootMotor = new NEO(Constants.RIGHT_SHOOT_MOTOR_ID);

    screwDriveMotor.configurePIDFF(Constants.SCREW_KP, Constants.SCREW_KI, Constants.SCREW_KD);

    leftShootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
    rightShootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
  }

  public void runFeeder() {
    this.runFeeder(Constants.FEEDER_SPEED);
  }
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }
  public void stopFeeder() {
    this.runFeeder(0.0);
  }

  public double getScrewDriveRotations() {
    return Constants.SCREW_DRIVE_GEAR_RATIO * screwDriveMotor.getPosition();
  }

  public void runScrewDrive(double speed) {
    double rotations = this.getScrewDriveRotations();
    if (rotations <= 0 && speed < 0) { return; }
    if (rotations >= Constants.MAX_SCREW_DRIVE_ROTATIONS && speed > 0) { return; }
    screwDriveMotor.set(speed);
  }
  public void stopScrewDrive() {
    screwDriveMotor.set(0.0);
  }

  public void setScrewDrive(double rotations) {
    screwDriveMotor.setPosition(rotations);
  }
  public void setScrewDriveInches(double inches) {
    screwDriveMotor.setPosition(inches/10);
  }

  public void setShooterSpeed(double speed) {
    leftShootMotor.setVelocity(speed);
    rightShootMotor.setVelocity(speed);
    shootSpeed = speed;
  }
  public void runShooterPercent(double percent) {
    leftShootMotor.set(percent);
    rightShootMotor.set(percent);
  }
  public void stopShooter() {
    this.runShooterPercent(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
