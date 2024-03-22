// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BotAngleSetpoint extends Command {
  private final DrivetrainSubsystem m_subsystem;
  private final double angle;

  /** Creates a new BotAngleSetpoint. */
  public BotAngleSetpoint(DrivetrainSubsystem subsystem, double angle) {
    this.m_subsystem = subsystem;
    this.angle = angle;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runAngleSetpoint(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setAllToState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
