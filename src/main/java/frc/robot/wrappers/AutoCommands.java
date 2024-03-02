// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScrewSetpointCommand;
import frc.robot.commands.ShootWhenReady;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScrewSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class AutoCommands {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ScrewSubsystem m_screwSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final FeederSubsystem m_feederSubsystem;

  public AutoCommands(DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem,
      ScrewSubsystem screwSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_screwSubsystem = screwSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_feederSubsystem = feederSubsystem;
  }

  public Command ShootDrive() {
    return Commands.sequence(
        this.ShootDoNothing(),
        this.DriveOnly()
    );
  }

  public Command ShootDoNothing() {
    return Commands.sequence(
        new ScrewSetpointCommand(m_screwSubsystem, Constants.SPEAKER_SETPOINT),
        new ShootWhenReady(m_shooterSubsystem, m_feederSubsystem, m_intakeSubsystem, 1.0).withTimeout(2.5));
  }

  public Command DriveOnly() {
    return Commands.parallel(
      new ScrewSetpointCommand(m_screwSubsystem, Constants.SPEAKER_SETPOINT),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(4000, Rotation2d.fromDegrees(180.0)))
        .withTimeout(3)
        );
  }

  public Command DriveWithIntake() {
    return Commands.parallel(
      new IntakeCommand(m_intakeSubsystem),
      this.DriveOnly()
    ).withTimeout(4);
  }

  public Command ShootDriveIntake() {
    return Commands.sequence(
      this.ShootDoNothing(),
      this.DriveWithIntake()
    );
  }

  public Command TwoNote() {
    return Commands.sequence(
      this.ShootDriveIntake(),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(-4000, Rotation2d.fromDegrees(180.0)))
        .withTimeout(3.5),
      this.ShootDoNothing()
    );
  }

  public Command TwoNoteDriveOut() {
    return Commands.sequence(
      this.TwoNote(),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(5000, Rotation2d.fromDegrees(180.0)))
    );
  }
}
