// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.stream.IntStream;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ScrewSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooter extends Command {
  private final ScrewSubsystem m_subsystem;
  private final PhotonCamera m_camera;

  /** Creates a new AimShooter. */
  public AimShooter(ScrewSubsystem subsystem, PhotonCamera camera) {
    this.m_subsystem = subsystem;
    this.m_camera = camera;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_camera.getLatestResult();
    if (!result.hasTargets()) { return; }

    PhotonTrackedTarget bestTarget = result.getBestTarget();
    int id = bestTarget.getFiducialId();

    // does nothing if the best target is not a speaker april tag
    if (!IntStream.of(Constants.SPEAKER_IDS).anyMatch(x -> x == id)) { return; }

    double range = PhotonUtils.calculateDistanceToTargetMeters(
      Constants.CAMERA_HEIGHT_METERS, 
      Constants.SPEAKER_TARGET_HEIGHT_METERS, 
      Constants.CAMERA_PITCH_RADIANS, 
      Units.degreesToRadians(bestTarget.getPitch()));

    double shootAngle = Math.atan(Constants.SPEAKER_OPENING_TOP / range);
    double hingeAngle = m_subsystem.shooterAngleToHingeAngle(shootAngle);
    m_subsystem.runSetpoint(hingeAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
