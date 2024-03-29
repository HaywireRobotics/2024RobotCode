// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AimShooter;
import frc.robot.commands.AlignSpeaker;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.BotAngleSetpoint;
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
  private final PhotonCamera m_camera;

  public AutoCommands(DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem,
      ScrewSubsystem screwSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, PhotonCamera camera) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_screwSubsystem = screwSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_feederSubsystem = feederSubsystem;
    this.m_camera = camera;
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
        new ShootWhenReady(m_shooterSubsystem, m_feederSubsystem, m_intakeSubsystem, 0.6).withTimeout(1.5));
  }

  public Command ShootAngleDrivePositive() {
    return this.ShootAngleDrive(62);
  }
  public Command ShootAngleDriveNegative() {
    return this.ShootAngleDrive(-62);
  }
  public Command ShootAngleDrive(double angle) {
    return Commands.sequence(
      this.ShootDoNothing(),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(4000, Rotation2d.fromDegrees(180.0)))
        .withTimeout(0.3),
      new BotAngleSetpoint(m_drivetrainSubsystem, angle).withTimeout(1),
      this.DriveWithIntake()
    );
  }

  public Command TwoNoteAngle(double angle) {
    return Commands.sequence(
      this.ShootAngleDrive(angle),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(-4000, Rotation2d.fromDegrees(180.0)))
        .withTimeout(0.5),
      Commands.parallel(new AlignSpeaker(m_drivetrainSubsystem, m_camera), new AimShooter(m_screwSubsystem, m_shooterSubsystem, m_camera)).withTimeout(2.5),
      new ShootWhenReady(m_shooterSubsystem, m_feederSubsystem, m_intakeSubsystem, 1.0).withTimeout(2.5)
    );
  }

  public Command TwoNoteAnglePositive() {
    return this.TwoNoteAngle(62);
  }
  public Command TwoNoteAngleNegative() {
    return this.TwoNoteAngle(-62);
  }

  public Command DriveOnly() {
    return Commands.parallel(
      new ScrewSetpointCommand(m_screwSubsystem, Constants.SPEAKER_SETPOINT),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(4000, Rotation2d.fromDegrees(180.0)))
        .withTimeout(2.6)
        );
  }

  public Command DriveWithIntake() {
    return Commands.parallel(
      new IntakeCommand(m_intakeSubsystem),
      this.DriveOnly()
    ).withTimeout(3.2);
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

  public Command TwoNoteAiming() {
    return Commands.sequence(
      this.ShootDriveIntake(),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(-4000, Rotation2d.fromDegrees(180.0)))
        .withTimeout(0.5),
      Commands.parallel(new AlignSpeaker(m_drivetrainSubsystem, m_camera), new AimShooter(m_screwSubsystem, m_shooterSubsystem, m_camera)).withTimeout(2.5),
      new ShootWhenReady(m_shooterSubsystem, m_feederSubsystem, m_intakeSubsystem, 1.0).withTimeout(2.5)
    );
  }

  public Command TwoNoteDriveOut() {
    return Commands.sequence(
      this.TwoNote(),
      new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(5000, Rotation2d.fromDegrees(180.0)))
    );
  }
}
