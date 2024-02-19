package frc.robot.wrappers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

public class SwerveModule {

    private double INTEGRATOR_RANGE = 0.01;
    private double ROTATION_KP = 0.0048; // 0.003
    private double ROTATION_KI = 0.0025; //0.00  // 0.0015
    private double ROTATION_KD = 0.00001;
    private double ROTATION_KIZ = this.INTEGRATOR_RANGE;
    private double ROTATION_KFF = 0;
    private double DRIVE_KP = 0.00007; //6.5e-5;
    private double DRIVE_KI = 0.0;  //5.5e-7;
    private double DRIVE_KD = 0.0; //0.001;
    private double DRIVE_KIZ = 0;
    private double DRIVE_KFF = 0;

    private boolean enabled = false;

    private final double OFFSET;
    private double initialEncoderAngle = 0.0;

    private SwerveModuleState desiredState;

    private final NEO rotationMotor;
    private final NEO driveMotor;

    private final CANcoder rotationEncoder;

    /* Odometry */
    private double rotationAngle;
    private double driveAngle;
    private double pRotationAngle;
    private double pDriveAngle;
    private Vector deltaPosition = new Vector(0., 0.);

    public SwerveModule(int driveID, int rotationID, int encoderID, double offset, boolean reverseDrive) {
        this(new NEO(driveID, reverseDrive), new NEO(rotationID), new CANcoder(encoderID), offset);
    }

    public SwerveModule(NEO driveMotor, NEO rotationMotor, CANcoder rotationEncoder, double offset) {
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
        this.rotationEncoder = rotationEncoder;
        
        this.driveMotor.configurePIDFF(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KIZ, DRIVE_KFF);
        this.rotationMotor.configurePIDFF(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_KIZ, ROTATION_KFF);

        rotationEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1));


        this.OFFSET = offset;

        zeroEncoders();
    }

    public void setRotationPID(double kp, double ki, double kd) {
        ROTATION_KP = kp;
        ROTATION_KI = ki;
        ROTATION_KD = kd;
        rotationMotor.configurePIDFF(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_KIZ, ROTATION_KFF);
    }

    public void setDrivePID(double kp, double ki, double kd) {
        DRIVE_KP = kp;
        DRIVE_KI = ki;
        DRIVE_KD = kd;
        driveMotor.configurePIDFF(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KIZ, DRIVE_KFF);
    }

    public void setStateRPM(SwerveModuleState state) {
        desiredState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotationAbsolute()));

        if ( isEnabled() ){
            // If drive is stoped, hold last angle.
            double stateAngle = Double.isNaN(desiredState.angle.getDegrees()) ? pRotationAngle : desiredState.angle.getDegrees();
            double rotationTarget = this.getRotation() + angleDifference(this.getRotationAbsolute(), stateAngle);
            rotationMotor.setPosition(this.angleToSteerRotations(rotationTarget));

            double rotationError = rotationTarget - this.steerRotationsToAngle(rotationMotor.getPosition());
            double directionSmoothing = Math.cos(Math.toRadians(rotationError));
            driveMotor.setVelocity(desiredState.speedMetersPerSecond * directionSmoothing); // ACTUALLY EXPECTS RPMS, OUGHT TO FIX
        }
        
        SmartDashboard.putNumber("SVAngle"+rotationEncoder.getDeviceID(), getRotation());
    }
    public void setStateMetersPerSecond(SwerveModuleState state) {
        double metersPerSecond = Statics.rpmToMetersPerSecond(state.speedMetersPerSecond);
        SwerveModuleState newState = new SwerveModuleState(metersPerSecond, state.angle);
        this.setStateRPM(newState);
    }

    public void driveDirect(double driveSpeed, double rotationSpeed) {
        driveMotor.set(driveSpeed);
        rotationMotor.set(rotationSpeed);
    }

    public double getSpeedMetersPerSecond() {
        return Statics.rpmToMetersPerSecond(driveMotor.getVelocity());
    }

    public double getRotationAbsolute() {
        return (rotationEncoder.getAbsolutePosition().getValueAsDouble() - OFFSET) * 360;
    }
    public double getRawRotationAbsolute() {
        return rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
    // public double getNeoRotation(){
    //     return this.rotationMotor.getPosition()*encoderScale;
    // }
    public double getRotation() {
        return this.steerRotationsToAngle(this.rotationMotor.getPosition());
    }

    public double angleToSteerRotations(double angle) {
        return (angle - initialEncoderAngle / 360) * Constants.STEER_MOTOR_GEAR_RATIO;
    }
    public double steerRotationsToAngle(double rotations) {
        return (rotations / Constants.STEER_MOTOR_GEAR_RATIO) * 360 + initialEncoderAngle;
    }

    public void zeroEncoders(){
        rotationMotor.setEncoder(0);
        initialEncoderAngle = getRotationAbsolute();
    }

    public static double angleDifference( double angle1, double angle2 )
    {
        double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMetersPerSecond(), Rotation2d.fromDegrees(getRotation()));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public int getID() {
        return rotationEncoder.getDeviceID();
    }

    public void putRawRotationSmartDashboard() {
        SmartDashboard.putNumber("CANcoder " + getID() + " Raw Rotation", getRawRotationAbsolute());
    }

    public void putRotationSmartDashboard() {
        SmartDashboard.putNumber("CANcoder " + getID() + " Rotation", getRotationAbsolute());
    }

    public void putSpeedSmartDashboard() {
        SmartDashboard.putNumber("Module " + getID() + " Speed", Math.abs(getSpeedMetersPerSecond()));
    }

    /* Odometry */
    public void updateOdometry(){
        this.pDriveAngle = this.driveAngle;
        this.pRotationAngle = this.rotationAngle;

        this.rotationAngle = this.getRotationAbsolute();
        this.driveAngle = this.driveMotor.getPosition();

        double speed = (this.driveAngle - this.pDriveAngle);
        double direction = (this.pRotationAngle/2 + this.rotationAngle/2);

        this.deltaPosition.x = Math.sin(Math.toRadians(direction))*speed;
        this.deltaPosition.y =  Math.cos(Math.toRadians(direction))*speed;
    }

    public Vector getDeltaPosition(){
        return this.deltaPosition;
    }

    public Vector getVelocity(){
        return this.deltaPosition.scale(Constants.WHEEL_DIAMETER * Math.PI).scale(1/Constants.DRIVE_MOTOR_GEAR_RATIO);
    }

    public void disable(){
        enabled = false;
    }
    public void enable(){
        enabled = true;
    }
    public boolean isEnabled(){
        return enabled;
    }
}
