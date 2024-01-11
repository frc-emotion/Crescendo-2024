package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


/**
 * Note: Uses analog absolute encoder instead of a CANCoder
 * 
 */
public class SwerveModuleNeoFalcon {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final FeedbackConfigs talonFXConfigurator;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModuleNeoFalcon(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        talonFXConfigurator = new FeedbackConfigs().withFeedbackRemoteSensorID(FeedbackDevice.IntegratedSensor.value);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.getConfigurator().apply(talonFXConfigurator);
        turningMotor.getConfigurator().apply(talonFXConfigurator);

        // driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor.value);
        // turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return toMeter(toRot(driveMotor.getPosition().getValueAsDouble()));
    }

    public double getDriveVelocity() {
        return toMPS(toRPM(driveMotor.getVelocity().getValueAsDouble()));
    }

    public double getTurningPosition() {
        return toRad(toRot(turningMotor.getPosition().getValueAsDouble()));
    }

    public double getTurningVelocity() {
        return toRadPerSec(toRPM(turningMotor.getVelocity().getValueAsDouble()));
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveMotor.setPosition(0);
        turningMotor.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * May have to use velocity PID and a feedforward command to have more accurate
     * 
     * @param state
     */
    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.3){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

    // Conversion Factors
    private double toRot(double ticks) {
        return ticks / 2048;
    }

    private double toRPM(double ticks_per_time) {
        return (ticks_per_time / 2048) * 600;
    }

    private double toMeter(double rot) {
        return rot * (ModuleConstants.kDriveEncoderRot2Meter);
    }

    private double toRad(double rot) {
        return rot * (ModuleConstants.kTurningEncoderRot2Rad);
    }

    private double toMPS(double rpm) {
        return rpm * (ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    private double toRadPerSec(double rpm) {
        return rpm * (ModuleConstants.kTurningEncoderRPM2RadPerSec);
    }
}