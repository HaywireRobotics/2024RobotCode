// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ScrewSubsystem extends SubsystemBase {
  private final NEO screwMotor;
  private final DutyCycleEncoder hingeEncoder;

  // private final PIDController screwController;

  /** Creates a new ScrewSubsystem. */
  public ScrewSubsystem() {
    this.screwMotor = new NEO(Constants.SCREW_MOTOR_ID);
    this.hingeEncoder = new DutyCycleEncoder(Constants.HINGE_ENCODER_ID);
    hingeEncoder.setDistancePerRotation(360.0);

    screwMotor.configurePIDFF(Constants.SCREW_KP, Constants.SCREW_KI, Constants.SCREW_KD);
  }

  public void runScrew(double speed) {
    double angle = hingeEncoder.getAbsolutePosition();
    if (angle <= Constants.HINGE_MIN_ANGLE && speed > 0) { speed = 0; }
    if (angle >= Constants.HINGE_MAX_ANGLE && speed < 0) { speed = 0; }
    screwMotor.set(speed);
  }
  public void stopScrew() {
    screwMotor.set(0.0);
  }
  
  public void setPositionInches(double inches) {
    double rawRotations = this.inchesToRawRotations(inches);
    screwMotor.setPosition(rawRotations);
  }
  public void setPositionDegrees(double angle) {
    double inches = shooterAngleToInches(angle);
    this.setPositionInches(inches);
  }

  private double screwRotationsToRawRotations(double rotations) {
    return rotations * Constants.SCREW_GEAR_RATIO;
  }
  private double inchesToScrewRotations(double inches) {
    return inches / Constants.SCREW_ROTATIONS_PER_INCH;
  }
  private double inchesToRawRotations(double inches) {
    return this.screwRotationsToRawRotations(this.inchesToScrewRotations(inches));
  }

  // I promise the math is accurate, please don't touch -Eli
  private double inchesToShooterAngle(double inches) {
    double s = inches + Constants.EXTENSION_BEFORE_SCREW;
    double a = Math.sqrt(s*s + Math.pow(Constants.SCREW_HINGE_DROPDOWN, 2));
    double b = Math.sqrt(Math.pow(Constants.SHOOTER_HINGE_X, 2) + Math.pow(Constants.SCREW_MOTOR_HEIGHT, 2));
    double c = Constants.SHOOTER_LENGTH_TO_SCREW_HINGE;

    double alpha = Math.acos((b*b + c*c - a*a) / (2*b*c));
    double theta = 180 - Math.atan(Constants.SCREW_MOTOR_HEIGHT/Constants.SHOOTER_HINGE_X) - alpha;
    return theta;
  }
  private double shooterAngleToInches(double angle) {
    double b = Math.sqrt(Math.pow(Constants.SHOOTER_HINGE_X, 2) + Math.pow(Constants.SCREW_MOTOR_HEIGHT, 2));
    double c = Constants.SHOOTER_LENGTH_TO_SCREW_HINGE;

    double x = 2*b*c * Math.cos(180 - Math.atan(Constants.SCREW_MOTOR_HEIGHT/Constants.SHOOTER_HINGE_X) - angle);
    double s = Math.sqrt(b*b + c*c - Math.pow(Constants.SCREW_HINGE_DROPDOWN, 2) - x);
    double inches = s - Constants.EXTENSION_BEFORE_SCREW;
    return inches;
  }

  public double getScrewRotations() {
    return screwMotor.getPosition() / Constants.SCREW_GEAR_RATIO;
  }
  public double getScrewExtensionInches() {
    return this.getScrewRotations() * Constants.SCREW_ROTATIONS_PER_INCH;
  }

  public void setScrewRaw(double rawRotations) {
    screwMotor.setPosition(rawRotations);
  }
  public void setScrew(double rotations) {
    if (rotations < 0 || rotations > Constants.MAX_SCREW_ROTATIONS) { return; }
    screwMotor.setPosition(this.screwRotationsToRawRotations(rotations));
  }
  public void setScrewInches(double inches) {
    if (inches < 0 || inches > Constants.MAX_SCREW_INCHES) { return; }
    screwMotor.setPosition(this.inchesToRawRotations(inches));
  }

  @Override
  public void periodic() {
    double speed = screwMotor.getMotor().get();
    double angle = hingeEncoder.getAbsolutePosition();
    // if (angle <= Constants.HINGE_MIN_ANGLE && speed < 0) { screwMotor.set(0.0); }
    // if (angle >= Constants.HINGE_MAX_ANGLE && speed > 0) { screwMotor.set(0.0); }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Screw Drive Rotations", screwMotor.getPosition());
    SmartDashboard.putNumber("Screw Drive Extension Inches", this.getScrewExtensionInches());
    SmartDashboard.putNumber("Hinge Absolute Encoder", hingeEncoder.getAbsolutePosition());
  }
}
