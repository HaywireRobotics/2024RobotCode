// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedCommand extends CommandBase {
  private final FeederSubsystem m_subsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final boolean reversed;

  /** Creates a new FeedCommand. */
  public FeedCommand(FeederSubsystem subsystem, IntakeSubsystem intakeSubsystem) {
    this(subsystem, intakeSubsystem, false);
  }

  public FeedCommand(FeederSubsystem subsystem, IntakeSubsystem intakeSubsystem, boolean reversed) {
    this.m_subsystem = subsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.reversed = reversed;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!reversed) {
      m_subsystem.runFeeder(Constants.FEEDER_SPEED);
      // m_intakeSubsystem.driveIntake(Constants.INTAKE_SPEED);
    } else {
      m_subsystem.runFeeder(Constants.FEEDER_REVERSE_SPEED);
      // m_intakeSubsystem.driveIntake(-Constants.INTAKE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopFeeder();
    m_intakeSubsystem.driveIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
