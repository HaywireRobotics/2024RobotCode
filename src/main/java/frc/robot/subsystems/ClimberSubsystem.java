// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ClimberSubsystem extends SubsystemBase {
  private final NEO leftArmMotor;
  private final NEO rightArmMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.leftArmMotor = new NEO(Constants.LEFT_ARM_ID);
    this.rightArmMotor = new NEO(Constants.RIGHT_ARM_ID);
  }

  // make "righty-tighty" (never mind the fact that we don't actually know what that means in this context yet)
  public void runArms(double percent) {
    this.runLeftArm(percent);
    this.runRightArm(percent);
  }
  public void runLeftArm(double percent) {
    leftArmMotor.set(percent);
  }
  public void runRightArm(double percent) {
    leftArmMotor.set(-percent);
  }
  public void stopArms() {
    this.runArms(0.0);
  }

  public double getLeftRotations() {
    return leftArmMotor.getPosition();
  }
  public double getRightRotations() {
    return rightArmMotor.getPosition();
  }

  private double rotationsToMeters(double rotations) {
    return (rotations / Constants.CLIMBER_GEAR_RATIO) * Constants.PULLEY_CIRCUMFERENCE;
  }
  private double metersToRotations(double meters) {
    return (meters / Constants.PULLEY_CIRCUMFERENCE) * Constants.CLIMBER_GEAR_RATIO;
  }
  public double getLeftHeight() {
    return this.rotationsToMeters(leftArmMotor.getPosition());
  }
  public double getRightHeight() {
    return this.rotationsToMeters(-this.rightArmMotor.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Arm Rotations", getLeftRotations());
    SmartDashboard.putNumber("Right Arm Rotations", getRightRotations());
    SmartDashboard.putNumber("Left Arm Height", getLeftHeight());
    SmartDashboard.putNumber("Right Arm Height", getRightHeight());
  }
}
