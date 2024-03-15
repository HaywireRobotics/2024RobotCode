// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ScrewSubsystem;
import frc.robot.util.Statics;

public class ScrewSetpointCommand extends Command {
  private final ScrewSubsystem m_subsystem;
  private final double setpoint;

  /** Creates a new ScrewSetpointCommand. */
  public ScrewSetpointCommand(ScrewSubsystem subsystem, double setpoint) {
    this.m_subsystem = subsystem;
    this.setpoint = setpoint;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopScrew();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isReady();
  }
}
