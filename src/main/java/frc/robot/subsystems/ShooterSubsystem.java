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
  private final NEO leftShootMotor;
  private final NEO rightShootMotor;

  private double shootSpeed = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.leftShootMotor = new NEO(Constants.LEFT_SHOOT_MOTOR_ID);
    this.rightShootMotor = new NEO(Constants.RIGHT_SHOOT_MOTOR_ID);

    leftShootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
    rightShootMotor.configurePIDFF(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD);
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
  }
}
