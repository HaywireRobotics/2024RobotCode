// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScrewSubsystem;

public class ScrewSetpointCommand extends Command {
  private final ScrewSubsystem m_subsystem;
  private double setpoint;

  private final int pointsUntilReady = 75;
  private List<Double> data = new ArrayList<Double>();
  private double average = 0.0;

  /** Creates a new ScrewSetpointCommand. */
  public ScrewSetpointCommand(ScrewSubsystem subsystem, double setpoint) {
    this.m_subsystem = subsystem;
    this.setpoint = setpoint;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_subsystem.getHingeAngle();
    double calc = m_subsystem.hingeController.calculate(angle, setpoint);
    m_subsystem.runScrew(-calc);
    
    addDatapoint(data, m_subsystem.getHingeAngle());
    average = calculateAverage(data);
  }
  
  public boolean isReady() {
    return Statics.isWithinError(average, setPoint, Constants.HINGE_MARGIN_OF_ERROR) &&
           Statics.isWithinError(m_subsystem.getHingeAngle(), setPoint, Constants.HINGE_MARGIN_OF_ERROR);
  }
  private final List<Double> addDatapoint(List<Double> list, Double datapoint) {
    list.add(datapoint.doubleValue());
    if (list.size() <= this.pointsUntilReady) {
       return list;
    } else {
       list.remove(0);
       return list;
    }
  }
  private final double calculateAverage(List<Double> list) {
    if (list.isEmpty()) { return 0.0; }
    double sum = 0.0;
    for (Double value : list) {
      sum += value;
    }
    return sum / list.size();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopScrew();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReady();
  }
}
