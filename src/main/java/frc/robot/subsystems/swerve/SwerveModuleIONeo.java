package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Most methods in this class have been ported over from the old SwerveModuleNeo class. Should be updated. */
public class SwerveModuleIONeo implements SwerveModuleIO {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder, turningEncoder;
    private final SparkPIDController turningPidController;

    private MagnetSensorConfigs magnetConfiguration;

    private final CANcoder absoluteEncoder;
    // private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private double resetIteration = 0;
    // private double referenceAngleRadians = 0;

    private final int ENCODER_RESET_ITERATIONS = 500;

    private boolean station;
    private SwerveModuleState desiredModuleState;
    

    public SwerveModuleIONeo(
        int driveMotorId,
            int turningMotorId,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderId,
            double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        // this.absoluteEncoderReversed = absoluteEncoderReversed;

        magnetConfiguration = new MagnetSensorConfigs();

        magnetConfiguration.withMagnetOffset(absoluteEncoderOffsetRad);
        magnetConfiguration.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        magnetConfiguration.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        absoluteEncoder = new CANcoder(absoluteEncoderId);
        absoluteEncoder.getConfigurator().apply(magnetConfiguration);
        absoluteEncoder.getPosition().setUpdateFrequency(absoluteEncoder.getPosition().getAppliedUpdateFrequency() / 4);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

        driveMotor.setSmartCurrentLimit(45);
        driveMotor.setSecondaryCurrentLimit(45);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        turningMotor.setSmartCurrentLimit(45);
        turningMotor.setSecondaryCurrentLimit(45);
        turningMotor.setInverted(turningMotorReversed);
        turningMotor.setIdleMode(IdleMode.kBrake);

        turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(
                ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(
                ModuleConstants.kTurningEncoderRPM2RadPerSec);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(
                ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(
                ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningPidController = turningMotor.getPIDController();

        resetEncoders();

        turningPidController.setP(DriveConstants.MODULE_kP);
        turningPidController.setI(DriveConstants.MODULE_kI);
        turningPidController.setD(DriveConstants.MODULE_kD);
        turningMotor.enableVoltageCompensation(12.0);

        station = false;
        desiredModuleState = new SwerveModuleState();
    }

    @Override
    public void updatePID(double kP, double kI, double kD) {
        turningPidController.setP(kP);
        turningPidController.setI(kI);
        turningPidController.setD(kD);
    }

    /** Updates the inputs in the IO Layer */
    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentModuleState = getState();
        inputs.desiredModuleState = desiredModuleState;
        inputs.currentModulePosition = getPosition();
        inputs.drivePosition = getDrivePosition();
        inputs.driveSpeed = getDriveVelocity();
        inputs.station = station;
        inputs.turnPosition = getTurningPosition();
        inputs.turnSpeed = getTurningVelocity();
    }

    /** Sends a raw voltage to the drive motor */
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    /** Retrieves the conversion factor for the position of the drive encoder */
    public double getPositionConversionFactor() {
        return driveEncoder.getPositionConversionFactor();
    }

    /** Sends a raw speed to the drive motor [-1, 1] */
    public void setRawDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    /** Retrieves the position of drive motor's encoder */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** Retrieves the velocity of the drive motor's encoder */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /** Retrieves the position of the turning motor's relative encoder */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /** Retrieves the update frequency of the absolute encoder */
    public double getAbsoluteEncoderUpdateFrequency() {
        return absoluteEncoder.getPosition().getAppliedUpdateFrequency();
    }

    /** Sets the update freqency of the absolute encoder */
    @Override
    public void setUpdateFrequency(double frequency) {
        absoluteEncoder.getPosition().setUpdateFrequency(frequency);
    }

    /** Retrieves the turning velocity of the turning motor's relative encoder */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /** Retrieves the current swerve module position */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                new Rotation2d(getTurningPosition()));
    }

    /** Retrieves the angle of the absolute encoder  */
    public double getAbsolutePostion() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    /** Sets the target angle for the turning motor */
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = getTurningPosition();

        if (getTurningVelocity() < Math.toRadians(0.5)) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = Units.degreesToRadians(getAbsolutePostion());// absoluteEncoder.getAbsolutePosition().getValueAsDouble()
                                                                                    // * 2.0 * Math.PI;
                turningEncoder.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        double adjustedReferenceAngleRadians = referenceAngleRadians +
                currentAngleRadians -
                currentAngleRadiansMod;

        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        // this.referenceAngleRadians = referenceAngleRadians;

        turningPidController.setReference(
                adjustedReferenceAngleRadians,
                CANSparkMax.ControlType.kPosition);
    }

    /** Sets the target speed for the drive motor */
    public void setSpeed(SwerveModuleState state, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveMotor.set(
                    state.speedMetersPerSecond /
                            DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        } else {
            // double velocity = MPSToFalcon(state.speedMetersPerSecond,
            // (((ModuleConstants.kWheelDiameterMeters / 2)) * 2 * Math.PI), 1 /
            // (ModuleConstants.kDriveMotorGearRatio));
            // driveMotor.set(ControlMode.Velocity, velocity,
            // DemandType.ArbitraryFeedForward,
            // feedforward.calculate(state.speedMetersPerSecond));
        }
    }

    /** Resets the drive encoder to 0 and the turning encoder to the position of the absolute encoder */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(
                Units.degreesToRadians(getAbsolutePostion()));
    }

    /** Retrieves the current swerve module state  */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(),
                new Rotation2d(getTurningPosition()));
    }

    /**
     * May have to use velocity PID and a feedforward command to have more accurate
     * driving.
     */
    @Override
    public void setDesiredModuleState(SwerveModuleState state, boolean station) {
        this.station = station;
        if (Math.abs(state.speedMetersPerSecond) < 0.01 && !station) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        desiredModuleState = state;
        // driveMotor.set(state.speedMetersPerSecond /
        // DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        setSpeed(state, true);
        // turningMotor.set(turningPidController.calculate(getTurningPosition(),
        // state.angle.getRadians()));
        setReferenceAngle(state.angle.getRadians());
        // Add Debug Here
    }

    /** Stops the drive and turning motors */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
