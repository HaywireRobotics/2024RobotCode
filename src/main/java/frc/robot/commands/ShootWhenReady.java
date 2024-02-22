// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWhenReady extends Command {
  private final ShooterSubsystem m_subsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final double speed;

  /** Creates a new ShootWhenReady. */
  public ShootWhenReady(ShooterSubsystem subsystem, FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem, double speed) {
    this.m_subsystem = subsystem;
    this.m_feederSubsystem = feederSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.speed = speed;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setShooterSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.isReady()) {
      m_feederSubsystem.runFeeder();
      m_intakeSubsystem.driveIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopShooter();
    m_feederSubsystem.stopFeeder();
    m_intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
