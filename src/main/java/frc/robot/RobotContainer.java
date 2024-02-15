// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoDriveState;
import frc.robot.commands.AutoFollowAprilTag;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScrewSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
  private final ScrewSubsystem m_screwSubsystem = new ScrewSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController m_driveController = new CommandXboxController(0);
  private final CommandXboxController m_manipulatorController = new CommandXboxController(1);

  // private final PhotonCamera m_camera = new PhotonCamera("banana");

  public final DefaultDriveCommand defaultDriveCommand;

  public RobotContainer() {
    defaultDriveCommand = new DefaultDriveCommand(m_drivetrainSubsystem, m_driveController);
    m_drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);

    // m_camera.setLED(VisionLEDMode.kOff);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveController.y().onTrue(new InstantCommand(m_drivetrainSubsystem::toggleFieldCentricDrive));
    m_driveController.b().onTrue(new InstantCommand(m_drivetrainSubsystem::zeroHeading));

    m_driveController.x().onTrue(new InstantCommand(() -> { m_drivetrainSubsystem.setDriveSpeed(800); }));
    m_driveController.a().onTrue(new InstantCommand(() -> { m_drivetrainSubsystem.setDriveSpeed(2000); }));

    m_driveController.leftBumper().onTrue(new IntakeCommand(m_intakeSubsystem));
    m_driveController.leftTrigger().onTrue(new IntakeCommand(m_intakeSubsystem, true));

    m_manipulatorController.leftTrigger().onTrue(new IntakeCommand(m_intakeSubsystem));
    m_manipulatorController.leftBumper().onTrue(new IntakeCommand(m_intakeSubsystem, true));
  }

  public void disable() {
    m_drivetrainSubsystem.disable();
  }
  public void enable() {
    m_drivetrainSubsystem.enable();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_drivetrainSubsystem.enable();

    // return new AutoDriveState(
    //   m_drivetrainSubsystem, 
    //   new SwerveModuleState(537, Rotation2d.fromDegrees(180.0))
    // ).withTimeout(7);

    // return new AutoFollowAprilTag(m_drivetrainSubsystem, m_camera);

    return new InstantCommand();
  }
}
