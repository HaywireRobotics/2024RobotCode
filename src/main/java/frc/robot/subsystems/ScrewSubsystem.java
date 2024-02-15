// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ScrewSubsystem extends SubsystemBase {
  private final NEO screwMotor;

  /** Creates a new ScrewSubsystem. */
  public ScrewSubsystem() {
    this.screwMotor = new NEO(Constants.SCREW_MOTOR_ID);

    screwMotor.configurePIDFF(Constants.SCREW_KP, Constants.SCREW_KI, Constants.SCREW_KD);
  }

  public void runScrew(double speed) {
    double rotations = this.getScrewRotations();
    if (rotations <= 0 && speed < 0) { return; }
    if (rotations >= Constants.MAX_SCREW_ROTATIONS && speed > 0) { return; }
    screwMotor.set(speed);
  }
  public void stopScrew() {
    screwMotor.set(0.0);
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Screw Drive Extension Inches", this.getScrewExtensionInches());
  }
}
