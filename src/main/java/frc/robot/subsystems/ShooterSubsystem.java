// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ShooterSubsystem extends SubsystemBase {
  private final NEO leftShootMotor;
  private final NEO rightShootMotor;

  private double setPoint = 0.0;
  private final int pointsUntilReady = 75;
  private List<Double> leftData = new ArrayList<Double>();
  private List<Double> rightData = new ArrayList<Double>();
  private double leftAverage = 0.0;
  private double rightAverage = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.leftShootMotor = new NEO(Constants.LEFT_SHOOT_MOTOR_ID);
    this.rightShootMotor = new NEO(Constants.RIGHT_SHOOT_MOTOR_ID);

    leftShootMotor.setIdleMode(IdleMode.kCoast);
    rightShootMotor.setIdleMode(IdleMode.kCoast);

    leftShootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
    rightShootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
  }

  public void setShooterSpeed(double speed) {
    leftShootMotor.setVelocity(-speed);
    rightShootMotor.setVelocity(speed);
    setPoint = speed;
  }
  public void runShooterPercent(double percent) {
    leftShootMotor.set(-percent);
    rightShootMotor.set(percent);
  }
  public void stopShooter() {
    this.runShooterPercent(0.0);
  }

  public boolean isReady() {
    return isReadyLeft() && isReadyRight();
  }
  public boolean isReadyLeft() {
    return isWithinMargin(leftAverage, setPoint, Constants.SHOOT_MARGIN_OF_ERROR) &&
           isWithinMargin(leftShootMotor.getVelocity(), setPoint, Constants.SHOOT_MARGIN_OF_ERROR);
  }
  public boolean isReadyRight() {
    return isWithinMargin(rightAverage, setPoint, Constants.SHOOT_MARGIN_OF_ERROR) &&
           isWithinMargin(rightShootMotor.getVelocity(), setPoint, Constants.SHOOT_MARGIN_OF_ERROR);
  }
  private boolean isWithinMargin(double value, double goal, double margin) {
    return value >= goal - margin && value <= goal + margin;
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

  @Override
  public void periodic() {
    addDatapoint(leftData, leftShootMotor.getVelocity());
    addDatapoint(rightData, rightShootMotor.getVelocity());
    leftAverage = calculateAverage(leftData);
    rightAverage = calculateAverage(rightData);

    SmartDashboard.putNumber("Left Shooter Speed", leftShootMotor.getVelocity());
    SmartDashboard.putNumber("Right Shooter Speed", rightShootMotor.getVelocity());
  }
}
