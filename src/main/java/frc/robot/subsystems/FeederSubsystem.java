// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class FeederSubsystem extends SubsystemBase {
  private final NEO feederMotor;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    this.feederMotor = new NEO(Constants.FEEDER_MOTOR_ID);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
