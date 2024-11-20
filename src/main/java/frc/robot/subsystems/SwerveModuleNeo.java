package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Note: Uses analog absolute encoder instead of a CANCoder
 *
 */
public class SwerveModuleNeo {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder, turningEncoder;
    private final SparkPIDController turningPidController, drivePIDController;

    private MagnetSensorConfigs magnetConfiguration;

    private final CANcoder absoluteEncoder;
    // private final boolean absoluteEncoderReversed;

    private double resetIteration = 0;
    // private double referenceAngleRadians = 0;

    private final int ENCODER_RESET_ITERATIONS = 500;

    public SwerveModuleNeo(
        int driveMotorId,
        int turningMotorId,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        int absoluteEncoderId,
        double absoluteEncoderOffset,
        boolean absoluteEncoderReversed
    ) {
        // this.absoluteEncoderReversed = absoluteEncoderReversed;

        magnetConfiguration = new MagnetSensorConfigs();

        magnetConfiguration.withMagnetOffset(absoluteEncoderOffset);
        magnetConfiguration.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        magnetConfiguration.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        absoluteEncoder = new CANcoder(absoluteEncoderId);
        absoluteEncoder.getConfigurator().apply(magnetConfiguration);
        absoluteEncoder.getPosition().setUpdateFrequency(50);
        absoluteEncoder.optimizeBusUtilization();

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

        driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveSmartCurrentLimit);
        driveMotor.setSecondaryCurrentLimit(ModuleConstants.kDriveSecondaryCurrentLimit);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setIdleMode(IdleMode.kBrake);
        
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        turningMotor.setSmartCurrentLimit(45);
        turningMotor.setSecondaryCurrentLimit(45);
        turningMotor.setInverted(turningMotorReversed);
        turningMotor.setIdleMode(IdleMode.kBrake);
        

        turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(
            ModuleConstants.kTurningEncoderRot2Deg);
        turningEncoder.setVelocityConversionFactor(
            ModuleConstants.kTurningEncoderRPM2DegPerSec);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(
            ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(
            ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningPidController = turningMotor.getPIDController();
        turningPidController.setFeedbackDevice(turningEncoder);
        turningPidController.setP(ModuleConstants.kPTurning);
        turningPidController.setI(ModuleConstants.kITurning);
        turningPidController.setD(ModuleConstants.kDTurning);

        turningPidController.setPositionPIDWrappingEnabled(true);
        turningPidController.setPositionPIDWrappingMinInput(-180);
        turningPidController.setPositionPIDWrappingMaxInput(180);
        turningPidController.setOutputRange(-1, 1);

        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(ModuleConstants.kPDrive);
        drivePIDController.setI(ModuleConstants.kIDrive);
        drivePIDController.setD(ModuleConstants.kDDrive);
        drivePIDController.setFeedbackDevice(driveEncoder);

        turningMotor.burnFlash();
        driveMotor.burnFlash();

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getTurningPosition() {
        return Rotation2d.fromDegrees(turningEncoder.getPosition());
    }

    public double getAbsoluteEncoderUpdateFrequency() {
        return absoluteEncoder.getPosition().getAppliedUpdateFrequency();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                getTurningPosition()
        );
    }

    public void setAngle(Rotation2d angle) {
        turningPidController.setReference(
            MathUtil.inputModulus(angle.getDegrees(), -180, 180),
            ControlType.kPosition
        );
    }

    public double getAbsolutePostion() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 180;
    }


    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(
                Units.degreesToRadians(getAbsolutePostion()));
    }

    public void calibrateAngle() {
        turningEncoder.setPosition(getAbsolutePostion());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(),
                getTurningPosition());
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getTurningPosition());

        setAngle(state.angle);
        setDrive(state.speedMetersPerSecond);
    }

    public void setDrive(double speed) {
        drivePIDController.setReference(speed, ControlType.kVelocity);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
