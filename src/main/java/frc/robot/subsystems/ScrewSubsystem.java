// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ScrewSetpointCommand;
import frc.robot.util.Statics;
import frc.robot.wrappers.NEO;

public class ScrewSubsystem extends SubsystemBase {
  private final NEO screwMotor;
  private final DutyCycleEncoder hingeEncoder;

  public final PIDController hingeController;

  private double setpoint;
  private final int pointsUntilReady = 75;
  private List<Double> data = new ArrayList<Double>();
  private double average = 0.0;

  /** Creates a new ScrewSubsystem. */
  public ScrewSubsystem() {
    this.screwMotor = new NEO(Constants.SCREW_MOTOR_ID);
    this.hingeEncoder = new DutyCycleEncoder(Constants.HINGE_ENCODER_ID);
    hingeEncoder.setDistancePerRotation(360.0);

    screwMotor.configurePIDFF(Constants.SCREW_KP, Constants.SCREW_KI, Constants.SCREW_KD);
    this.hingeController = new PIDController(Constants.HINGE_KP, Constants.HINGE_KI, Constants.HINGE_KD);
  }

  public double softStop(double speed) {
    double angle = hingeEncoder.getAbsolutePosition();
    if (angle <= Constants.HINGE_MIN_ANGLE && speed > 0) { return 0; }
    if (angle >= Constants.HINGE_MAX_ANGLE && speed < 0) { return 0; }
    return speed;
  }

  public void runScrew(double speed) {
    screwMotor.set(softStop(speed));
  }
  public void stopScrew() {
    screwMotor.set(0.0);
  }

  public void runSetpoint(double setpoint) {
    this.setpoint = setpoint;
    double angle = this.getHingeAngle();
    double calc = this.hingeController.calculate(angle, setpoint);
    double speed = softStop(-calc);
    this.runScrew(speed);
    
    addDatapoint(data, this.getHingeAngle());
    average = calculateAverage(data);
  }

  public boolean isReady() {
    return Statics.withinError(average, setpoint, Constants.HINGE_MARGIN_OF_ERROR) &&
           Statics.withinError(this.getHingeAngle(), setpoint, Constants.HINGE_MARGIN_OF_ERROR);
  }
  private final List<Double> addDatapoint(List<Double> list, Double datapoint) {
    list.add(datapoint.doubleValue());
    if (list.size() <= this.pointsUntilReady) {
       return list;
    } else {
       list.remove(0);
       return list;
    }
  }
  private final double calculateAverage(List<Double> list) {
    if (list.isEmpty()) { return 0.0; }
    double sum = 0.0;
    for (Double value : list) {
      sum += value;
    }
    return sum / list.size();
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
  // https://cdn.discordapp.com/attachments/1056488251529101382/1206832973115293746/IMG_20240212_222252469.jpg?ex=66025bca&is=65efe6ca&hm=e5578119ee5e6054464be6f516805161f55550b696392edb268803c26fa352b8&
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

  // private double shooterAngleToHingeAngle(double shooterAngle) {
  //   double alpha = 180 - shooterAngle - Math.atan(Constants.SCREW_MOTOR_HEIGHT / Constants.SHOOTER_HINGE_X);
  //   double b = Math.sqrt(Math.pow(Constants.SHOOTER_HINGE_X, 2) + Math.pow(Constants.SCREW_MOTOR_HEIGHT, 2));
  //   double c = Constants.SHOOTER_LENGTH_TO_SCREW_HINGE;

  //   double gamma = Math.asin( (c * Math.sin(alpha)) / Math.sqrt(b*b + c*c - 2*b*c*Math.cos(alpha)) );
  //   double s = 1.0; // TODO: calculate s
  //   double hingeAngle = gamma + Math.atan(Constants.SHOOTER_HINGE_X / Constants.SCREW_MOTOR_HEIGHT) + Math.atan(Constants.SCREW_HINGE_DROPDOWN / s);

  //   return hingeAngle;
  // }
  public double hingeAngleToShooterAngle(double hingeAngle) {
    return Constants.HINGE_SHOOTER_M*hingeAngle + Constants.HINGE_SHOOTER_B;
  }
  public double shooterAngleToHingeAngle(double shooterAngle) {
    return (shooterAngle - Constants.HINGE_SHOOTER_B) / Constants.HINGE_SHOOTER_M;
  }

  private Command shooterAngleSetpointCommand(double shooterAngle) {
    return new ScrewSetpointCommand(this, shooterAngleToHingeAngle(shooterAngle));
  }

  public double getScrewRotations() {
    return screwMotor.getPosition() / Constants.SCREW_GEAR_RATIO;
  }
  public double getScrewExtensionInches() {
    return this.getScrewRotations() * Constants.SCREW_ROTATIONS_PER_INCH;
  }

  public double getHingeAngle() {
    return this.hingeEncoder.getAbsolutePosition();
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
