// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.DefaultClimbCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScrewPercentCommand;
import frc.robot.commands.ScrewSetpointCommand;
import frc.robot.commands.ShootPercentCommand;
import frc.robot.commands.ShootWhenReady;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScrewSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.wrappers.AutoCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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

  private final AutoCommands m_autoCommands = new AutoCommands(m_drivetrainSubsystem, m_shooterSubsystem,
      m_screwSubsystem, m_intakeSubsystem, m_feederSubsystem);

  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  
  // private final PhotonCamera m_camera = new PhotonCamera("banana");

  public final DefaultDriveCommand defaultDriveCommand;
  public final DefaultClimbCommand defaultClimbCommand;

  public RobotContainer() {
    defaultDriveCommand = new DefaultDriveCommand(m_drivetrainSubsystem, m_driveController);
    m_drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);

    defaultClimbCommand = new DefaultClimbCommand(m_climberSubsystem, m_manipulatorController);
    m_climberSubsystem.setDefaultCommand(defaultClimbCommand);

    // m_camera.setLED(VisionLEDMode.kOff);

    m_autoChooser.setDefaultOption("Shoot Then Drive", m_autoCommands.ShootDrive());
    m_autoChooser.addOption("Shoot Do Nothing", m_autoCommands.ShootDoNothing());
    m_autoChooser.addOption("Autobots Role Out", m_autoCommands.DriveOnly());
    m_autoChooser.addOption("Drive With Intake", m_autoCommands.DriveWithIntake());
    m_autoChooser.addOption("Shoot Drive Intake", m_autoCommands.ShootDriveIntake());
    m_autoChooser.addOption("Two Note", m_autoCommands.TwoNote());
    m_autoChooser.addOption("Two Note Drive Again", m_autoCommands.TwoNoteDriveOut());
    m_autoChooser.addOption("Your Mother", new ScrewSetpointCommand(m_screwSubsystem, Constants.SPEAKER_SETPOINT));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveController.y().onTrue(new InstantCommand(m_drivetrainSubsystem::toggleFieldCentricDrive));
    m_driveController.b().onTrue(new InstantCommand(m_drivetrainSubsystem::zeroHeading));

    // m_driveController.x().onTrue(new InstantCommand(() -> {
    //   m_drivetrainSubsystem.setDriveSpeed(1800);
    // }));
    // m_driveController.a().onTrue(new InstantCommand(() -> {
    //   m_drivetrainSubsystem.setDriveSpeed(Constants.MAX_SPEED);
    // }));

    m_driveController.leftBumper().whileTrue(new IntakeCommand(m_intakeSubsystem));
    m_driveController.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, true));

    m_driveController.rightTrigger().onTrue(new InstantCommand(() -> {
      m_drivetrainSubsystem.setDriveSpeed(Constants.SLOW_SPEED);
    }));
    m_driveController.rightTrigger().onFalse(new InstantCommand(() -> {
      m_drivetrainSubsystem.setDriveSpeed(Constants.MAX_SPEED);
    }));
    m_driveController.rightBumper().onTrue(new InstantCommand(() -> {
      m_drivetrainSubsystem.setDriveSpeed(Constants.HYPER_SPEED);
    }));
    m_driveController.rightBumper().onFalse(new InstantCommand(() -> {
      m_drivetrainSubsystem.setDriveSpeed(Constants.MAX_SPEED);
    }));

    m_manipulatorController.leftBumper().whileTrue(new IntakeCommand(m_intakeSubsystem));
    m_manipulatorController.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, true));

    m_manipulatorController.rightBumper().whileTrue(new ShootPercentCommand(m_shooterSubsystem, 1.0));
    m_manipulatorController.rightTrigger().whileTrue(new ShootPercentCommand(m_shooterSubsystem, 0.6));
    m_manipulatorController.y().whileTrue(new FeedCommand(m_feederSubsystem, m_intakeSubsystem));
    m_manipulatorController.a().whileTrue(new FeedCommand(m_feederSubsystem, m_intakeSubsystem, true));

    m_manipulatorController.povDown().whileTrue(new ScrewPercentCommand(m_screwSubsystem, Constants.SCREW_SPEED));
    m_manipulatorController.povUp().whileTrue(new ScrewPercentCommand(m_screwSubsystem, -Constants.SCREW_SPEED));

    m_manipulatorController.povLeft().onTrue(new ScrewSetpointCommand(m_screwSubsystem, Constants.SPEAKER_SETPOINT));
    m_manipulatorController.povRight().onTrue(new ScrewSetpointCommand(m_screwSubsystem, Constants.AMP_SETPOINT));
  }

  public void disable() {
    m_drivetrainSubsystem.disable();
  }

  public void enable() {
    m_drivetrainSubsystem.enable();
    m_drivetrainSubsystem.zeroHeading();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_drivetrainSubsystem.enable();

    // return m_autoCommands.ShootDriveCommand();
    return m_autoChooser.getSelected();
  }
}
