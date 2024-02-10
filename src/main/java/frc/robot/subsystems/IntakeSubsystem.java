package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class IntakeSubsystem extends SubsystemBase {
    private final NEO leftMotor;
    private final NEO rightMotor;

    
    public IntakeSubsystem () {
        this.leftMotor = new NEO(Constants.LEFT_INTAKE_MOTOR_ID);
        this.rightMotor = new NEO(Constants.RIGHT_INTAKE_MOTOR_ID);
    }

    public void driveIntake(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }
}
