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
  private final NEO shootMotor;

  private double setPoint = 0.0;
  private final int pointsUntilReady = 75;
  private List<Double> leftData = new ArrayList<Double>();
  private double leftAverage = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.shootMotor = new NEO(Constants.SHOOT_MOTOR_ID);

    shootMotor.setIdleMode(IdleMode.kCoast);

    shootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
  }

  public void setShooterSpeed(double speed) {
    shootMotor.setVelocity(-speed);
    setPoint = speed;
  }
  public void runShooterPercent(double percent) {
    shootMotor.set(percent);
  }
  public void stopShooter() {
    this.runShooterPercent(0.0);
  }

  public boolean isReady() {
    return isWithinMargin(leftAverage, setPoint, Constants.SHOOT_MARGIN_OF_ERROR) &&
           isWithinMargin(shootMotor.getVelocity(), setPoint, Constants.SHOOT_MARGIN_OF_ERROR);
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
    addDatapoint(leftData, shootMotor.getVelocity());
    leftAverage = calculateAverage(leftData);

    SmartDashboard.putNumber("Left Shooter Speed", shootMotor.getVelocity());
  }
}
