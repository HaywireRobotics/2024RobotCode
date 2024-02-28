// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScrewSubsystem;

public class ScrewSetpointCommand extends Command {
  private final ScrewSubsystem m_subsystem;
  private double setpoint;

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
    double angle = m_subsystem.getHingeAngle();
    double calc = m_subsystem.hingeController.calculate(angle, setpoint);
    m_subsystem.runScrew(-calc);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopScrew();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
