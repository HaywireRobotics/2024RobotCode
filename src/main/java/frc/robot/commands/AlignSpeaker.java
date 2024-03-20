// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.stream.IntStream;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Statics;

public class AlignSpeaker extends Command {
  private final DrivetrainSubsystem m_subsystem;
  private final PhotonCamera m_camera;

  private final PIDController botRotationController = new PIDController(Constants.ROTATION_KP, Constants.ROTATION_KI, Constants.ROTATION_KD);
  private double botAngleSetpoint;

  /** Creates a new AimBot. */
  public AlignSpeaker(DrivetrainSubsystem subsystem, PhotonCamera camera) {
    this.m_subsystem = subsystem;
    this.m_camera = camera;

    this.botAngleSetpoint = m_subsystem.getNavx();

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.runSetpoint();

    var result = m_camera.getLatestResult();
    if (!result.hasTargets()) { return; }

    PhotonTrackedTarget centerTarget = result.getBestTarget();
    int id = centerTarget.getFiducialId();

    // does nothing if the best target is not a speaker april tag
    if (!IntStream.of(Constants.SPEAKER_CENTER_IDS).anyMatch(x -> x == id)) { return; }

    // List<PhotonTrackedTarget> targets = result.getTargets();
    // PhotonTrackedTarget centerTarget = new PhotonTrackedTarget(0, 0, 0, 0, 0, null, null, 0, null, null);
    // for (int i = 0; i < targets.size(); i++) {
    //   int id = targets.get(i).getFiducialId();
    //   if (IntStream.of(Constants.SPEAKER_CENTER_IDS).anyMatch(x -> x == id)) {
    //     centerTarget = targets.get(i);
    //     break;
    //   }
    //   if (i == targets.size() - 1) { return; }
    // }

    double relativeX = centerTarget.getBestCameraToTarget().getX();
    double relativeY = centerTarget.getBestCameraToTarget().getY();
    double rotationRelativeToBot = Math.toDegrees(Math.atan(relativeY / relativeX));
    botAngleSetpoint = m_subsystem.getNavx() + rotationRelativeToBot;
  }

  private void runSetpoint() {
    double error = Statics.calculateAngleError(m_subsystem.getNavx(), botAngleSetpoint);

    double calc = botRotationController.calculate(error, 0);
    m_subsystem.driveVector(0, 0, calc);
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
