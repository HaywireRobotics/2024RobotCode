package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveState extends Command {
    private final DrivetrainSubsystem m_subsystem;
    private final SwerveModuleState state;

    public AutoDriveState(DrivetrainSubsystem subsystem, SwerveModuleState state) {
        this.m_subsystem = subsystem;
        this.state = state;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.setAllToState(state);

        m_subsystem.updateOdometry();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setAllToState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
