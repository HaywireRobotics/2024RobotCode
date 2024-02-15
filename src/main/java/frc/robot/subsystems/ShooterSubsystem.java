// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void runScrewDrive(double speed) {
    double rotations = this.getScrewDriveRotations();
    if (rotations <= 0 && speed < 0) { return; }
    if (rotations >= Constants.MAX_SCREW_DRIVE_ROTATIONS && speed > 0) { return; }
    screwDriveMotor.set(speed);
  }
  public void stopScrewDrive() {
    screwDriveMotor.set(0.0);
  }

  private double screwRotationsToRawRotations(double rotations) {
    return rotations * Constants.SCREW_DRIVE_GEAR_RATIO;
  }
  private double inchesToScrewRotations(double inches) {
    return inches / Constants.SCREW_ROTATIONS_PER_INCH;
  }
  private double inchesToRawRotations(double inches) {
    return this.screwRotationsToRawRotations(this.inchesToScrewRotations(inches));
  }

  // I promise the math is accurate, please don't touch -Eli
  private double inchesToShooterAngle(double inches) {
    double s = inches + 7;
    double a = Math.sqrt(inches*inches + 1.9*1.9);
    double b = Math.sqrt(4*4 + 7.5*7.5);
    double c = 14;

    double alpha = Math.acos((b*b + c*c - a*a) / (2*b*c));
    double theta = 180 - Math.atan(7.5/4) - alpha;
    return theta;
  }
  private double shooterAngleToInches(double angle) {
    double b = Math.sqrt(4*4 + 7.5*7.5);
    double c = 14;

    double x = 2*b*c * Math.cos(180 - Math.atan(7.5/4) - angle);
    double s = Math.sqrt(b*b + c*c - 1.9*1.9 - x);
    double inches = s - 7;
    return inches;
  }

  public double getScrewDriveRotations() {
    return screwDriveMotor.getPosition() / Constants.SCREW_DRIVE_GEAR_RATIO;
  }
  public double getScrewDriveExtensionInches() {
    return this.getScrewDriveRotations() * Constants.SCREW_ROTATIONS_PER_INCH;
  }

  public void setScrewDriveRaw(double rawRotations) {
    screwDriveMotor.setPosition(rawRotations);
  }
  public void setScrewDrive(double rotations) {
    if (rotations < 0 || rotations > Constants.MAX_SCREW_DRIVE_ROTATIONS) { return; }
    screwDriveMotor.setPosition(this.screwRotationsToRawRotations(rotations));
  }
  public void setScrewDriveInches(double inches) {
    if (inches < 0 || inches > Constants.MAX_SCREW_DRIVE_INCHES) { return; }
    screwDriveMotor.setPosition(this.inchesToRawRotations(inches));
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

    SmartDashboard.putNumber("Left Shooter Speed", leftShootMotor.getVelocity());
    SmartDashboard.putNumber("Right Shooter Speed", rightShootMotor.getVelocity());
    SmartDashboard.putNumber("Screw Drive Extension Inches", this.getScrewDriveExtensionInches());
  }
}
