// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.stream.IntStream;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ScrewSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Statics;

public class AimShooter extends Command {
  private final ScrewSubsystem m_subsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final PhotonCamera m_camera;

  private double hingeAngleSetpoint = Constants.SPEAKER_SETPOINT;

  /** Creates a new AimShooter. */
  public AimShooter(ScrewSubsystem subsystem, ShooterSubsystem shooterSubsystem, PhotonCamera camera) {
    this.m_subsystem = subsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_camera = camera;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runSetpoint(hingeAngleSetpoint);
    SmartDashboard.putBoolean("Shoot Is Ready", m_subsystem.isReady());

    var result = m_camera.getLatestResult();
    if (!result.hasTargets()) { return; }

    PhotonTrackedTarget centerTarget = result.getBestTarget();
    // int id = centerTarget.getFiducialId();

    // does nothing if the best target is not a speaker april tag
    // if (!IntStream.of(Constants.SPEAKER_CENTER_IDS).anyMatch(x -> x == id)) { return; }

    List<PhotonTrackedTarget> targets = result.getTargets();
    for (int i = 0; i < targets.size(); i++) {
      int targetId = targets.get(i).getFiducialId();
      if (IntStream.of(Constants.SPEAKER_CENTER_IDS).anyMatch(x -> x == targetId)) {
        centerTarget = targets.get(i);
        break;
      }
      if (i == targets.size() - 1) { return; }
    }

    double range =  centerTarget.getBestCameraToTarget().getX();
    double x_distance = range * Math.cos(Constants.CAMERA_PITCH_RADIANS) - Constants.SHOOTER_CAMERA_OFFSET_METERS;

    SmartDashboard.putNumber("distance to target meters", x_distance);

    double y_goal_adjustment = Statics.clamp((x_distance - 1.8*1.285) * 0.5, 0, 1);
    SmartDashboard.putNumber("goal adjustment", y_goal_adjustment);
    // double y_distance = (Constants.SPEAKER_TIPPY_TOP_METERS - Constants.SHOOTER_HINGE_ELEVATION_METERS) + y_goal_adjustment;
    double y_distance = ((Constants.SPEAKER_TIPPY_TOP_METERS + Constants.SPEAKER_OPENING_TOP)/2 - Constants.SHOOTER_HINGE_ELEVATION_METERS) + y_goal_adjustment;
    // double y_distance = (Constants.SPEAKER_OPENING_TOP + Constants.SPEAKER_TIPPY_TOP_METERS) / 2;

    // double velocity = m_shooterSubsystem.getWheelSpeedMetersPerSecond();
    // double shootAngle = Math.toDegrees(Math.atan((2 * y_distance) / (Constants.TYPICAL_SPEED * x_distance)));

    double shootAngle = Math.toDegrees(Math.atan(y_distance / (x_distance)));
    SmartDashboard.putNumber("aim angle", shootAngle);
    hingeAngleSetpoint = m_subsystem.shooterAngleToHingeAngle(shootAngle);
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
