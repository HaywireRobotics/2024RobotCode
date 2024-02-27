// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.Statics;

public class DefaultClimbCommand extends Command {
  private final ClimberSubsystem m_subsystem;
  private final CommandXboxController m_controller;

  private static final double JOYSTICK_DEADBAND = 0.1;

  /** Creates a new DefaultClimbCommand. */
  public DefaultClimbCommand(ClimberSubsystem subsystem, CommandXboxController controller) {
    this.m_subsystem = subsystem;
    this.m_controller = controller;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightY = -m_controller.getRightY();
    double filteredRight = Statics.applyDeadband(rightY, JOYSTICK_DEADBAND);

    double leftY = -m_controller.getLeftY();
    double filteredLeft = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);

    double rightSpeed = filteredRight * Constants.CLIMBER_SPEED;
    double leftSpeed = filteredLeft * Constants.CLIMBER_SPEED;
    m_subsystem.runRightArm(rightSpeed);
    m_subsystem.runLeftArm(leftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArms();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
