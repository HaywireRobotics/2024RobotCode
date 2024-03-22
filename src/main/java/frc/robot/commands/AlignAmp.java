// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.stream.IntStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Statics;

public class AlignAmp extends Command {
  private final DrivetrainSubsystem m_subsystem;
  private final PhotonCamera m_camera;

  private final PIDController translationController = new PIDController(Constants.TRANSLATION_KP, Constants.TRANSLATION_KI, Constants.TRANSLATION_KD);
  private final PIDController botRotationController = new PIDController(Constants.ROTATION_KP, Constants.ROTATION_KI, Constants.ROTATION_KD);

  private double botAngleSetpoint;

  /** Creates a new AlignAmp. */
  public AlignAmp(DrivetrainSubsystem subsystem, PhotonCamera camera) {
    this.m_subsystem = subsystem;
    this.m_camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.botAngleSetpoint = m_subsystem.getNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_camera.getLatestResult();
    if (!result.hasTargets()) { return; }

    PhotonTrackedTarget centerTarget = result.getBestTarget();
    int id = centerTarget.getFiducialId();

    // does nothing if the best target is not a speaker april tag
    if (!IntStream.of(Constants.AMP_IDS).anyMatch(x -> x == id)) { return; }

    double translationError = centerTarget.getBestCameraToTarget().getY();
    double translationCalc = translationController.calculate(translationError, 0);
    double driveSpeed = m_subsystem.currentDriveSpeed * translationCalc;

    // double relativeX = centerTarget.getBestCameraToTarget().getX();
    // double relativeY = centerTarget.getBestCameraToTarget().getY();
    // double rotationRelativeToBot = Math.toDegrees(Math.atan(relativeY / relativeX));
    // double botRotationSetpoint = m_subsystem.getNavx() + rotationRelativeToBot;
    // double rotationError = Statics.calculateAngleError(m_subsystem.getNavx(), botRotationSetpoint);
    // double rotationCalc = botRotationController.calculate(rotationError, 0);
    // double rotationZ = centerTarget.getBestCameraToTarget().getRotation().getZ();
    // double relativeToBot;
    // if (rotationZ <= 0) {
    //   relativeToBot = -180 - rotationZ;
    // } else {
    //   relativeToBot = 180 - rotationZ;
    // }
    double relativeToBot = -Statics.calculateAngleError(centerTarget.getBestCameraToTarget().getRotation().getZ(), 0);  // to account for strange results
    botAngleSetpoint = m_subsystem.getNavx() + relativeToBot;
    double rotationError = Statics.calculateAngleError(m_subsystem.getNavx(), botAngleSetpoint);
    double rotationCalc = botRotationController.calculate(rotationError, 0);

    double ampCentricAdjustment = -(m_subsystem.getNavx() + relativeToBot);
    double direction = 90 + ampCentricAdjustment;

    m_subsystem.driveVector(driveSpeed, direction, rotationCalc);
    // m_subsystem.driveVector(0, 0, rotationCalc);
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
