package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class IntakeSubsystem extends SubsystemBase {
    private final NEO topMotor;
    private final NEO bottomMotor;

    
    public IntakeSubsystem () {
        this.topMotor = new NEO(Constants.TOP_INTAKE_MOTOR_ID);
        this.bottomMotor = new NEO(Constants.BOTTOM_INTAKE_MOTOR_ID);

        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);
    }

    public void driveIntake() {
        this.driveIntake(Constants.INTAKE_SPEED);
    }
    public void driveIntake(double speed) {
        topMotor.set(-speed);
        bottomMotor.set(-speed);
    }

    public void stopIntake() {
        this.driveIntake(0.0);
    }
}
