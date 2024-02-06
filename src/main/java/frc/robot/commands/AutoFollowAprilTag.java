package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

// import edu.wpi.first.util.protobuf.ProtobufSerializable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoFollowAprilTag extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final PhotonCamera m_camera;

    public AutoFollowAprilTag(DrivetrainSubsystem subsystem, PhotonCamera camera) {
        this.m_subsystem = subsystem;
        this.m_camera = camera;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        // if (result == null) { m_subsystem.restDrive(); return; }
        if (!result.hasTargets()) { m_subsystem.restDrive(); return; }

        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() != 1) { continue; }

            List<TargetCorner> corners = target.getDetectedCorners();
            TargetCorner center = this.centerOfCorners(corners);

            SmartDashboard.putNumberArray("center", new double[]{center.x, center.y});
            
        }
    }

    private TargetCorner centerOfCorners(List<TargetCorner> corners) {
        double totalX = 0.0;
        double totalY = 0.0;
        for (TargetCorner targetCorner : corners) {
            totalX += targetCorner.x;
            totalY += targetCorner.y;
        }
        double x = totalX / corners.size();
        double y = totalY / corners.size();
        return new TargetCorner(x, y);
    }
}
