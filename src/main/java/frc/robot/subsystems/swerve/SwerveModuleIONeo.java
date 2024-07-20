package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants;

public class SwerveModuleIONeo implements SwerveModuleIO {
    private CANSparkMax turningMotor, driveMotor;
    private RelativeEncoder turningEncoder, driveEncoder;
    private CANcoder absoluteEncoder;

    private SparkPIDController driveController;

    private PIDController turnController;
    private SimpleMotorFeedforward driveFeedForward;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    public SwerveModuleIONeo(int id) {
        if(id < 0 || id > 3) {
            throw new IndexOutOfBoundsException("Swerve ID " + id + " is out of bounds of length 4.");
        }
        driveMotor = new CANSparkMax(DriveConstants.DRIVE_PORTS[id], MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        driveMotor.setSecondaryCurrentLimit(DriveConstants.kCurrentLimit);
        driveMotor.setInverted(DriveConstants.DRIVE_REVERSED[id]);
        driveMotor.setIdleMode(IdleMode.kCoast);

        driveController = driveMotor.getPIDController();
        driveController.setP(ModuleConstants.kPDrive);
        driveController.setI(ModuleConstants.kIDrive);
        driveController.setD(ModuleConstants.kDDrive);
        driveController.setOutputRange(-1, 1);

        driveEncoder = driveMotor.getEncoder();
        // driveEncoder.setInverted(DriveConstants.DRIVE_REVERSED[id]);

        turningMotor = new CANSparkMax(DriveConstants.TURNING_PORTS[id], MotorType.kBrushless);
        turningMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        turningMotor.setSecondaryCurrentLimit(DriveConstants.kSmartCurrentLimit);
        turningMotor.setInverted(DriveConstants.TURNING_REVERSED[id]);
        turningMotor.setIdleMode(IdleMode.kCoast);

        turnController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turnController.enableContinuousInput(-180, 180);

        turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Deg);

        // turningEncoder.setInverted(DriveConstants.TURNING_REVERSED[id]);

        absoluteEncoder = new CANcoder(DriveConstants.CANCODER_PORTS[id]);

        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs()
            .withMagnetOffset(DriveConstants.ENCODER_OFFSETS[id])
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);


        absoluteEncoder.getConfigurator().apply(magnetConfigs);
        absoluteEncoder.getPosition().setUpdateFrequency(ModuleConstants.CANCODER_UPDATE_FREQUENCY);

        //absoluteEncoder.setPosition(absoluteEncoder.getAbsolutePosition().getValue());

        driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSDrive, ModuleConstants.kVDrive);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.currentModulePosition = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAbsolutePosition()));
        inputs.currentModuleState = new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsolutePosition()));
        inputs.desiredModuleState = lastDesiredState;
        inputs.drivePosition = getDrivePosition();
        inputs.driveSpeed = getDriveVelocity();
        inputs.turnPosition = getAbsolutePosition();
        inputs.turnSpeed = getTurnVelocity();
        inputs.absolutePosition = getAbsolutePosition();
        inputs.station = true;
    }

    @Override
    public void updateConstants(double kPTurning, double kITurning, double kDTurning, double kPDrive, double kIDrive, double kDDrive) {
        turnController.setPID(kPTurning, kITurning, kDTurning);
        driveController.setP(kPDrive);
        driveController.setI(kIDrive);
        driveController.setD(kDDrive);
    }

    @Override
    public void setDesiredModuleState(SwerveModuleState state) {
        var optstate = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getTurnDegrees_180()));
        // setReferenceAngle(state.angle.getDegrees());
        lastDesiredState = state;

        setTurnPosition(optstate.angle.getDegrees());
        setDriveVelocity(optstate.speedMetersPerSecond);
    }

    @Override
    public void setReferenceAngle(double angle) {
        double currentAngle = getTurnDegrees_180();

        double oppositeAngle = (currentAngle + 180) % 360.0;

        double diff1 = angle - currentAngle;
        double diff2 = oppositeAngle - angle;

        if(Math.abs(diff1) < Math.abs(diff2)) {
            setTurnPosition(currentAngle + diff1);
        } else {
            setTurnPosition(currentAngle + diff2);
        }
    }

    private void setDriveVelocity(double velocityMPS) {
        driveController.setReference(velocityMPS / ModuleConstants.kDriveEncoderRPM2MeterPerSec, ControlType.kVelocity);
    }

    private void setTurnPosition(double degrees) {
        turningMotor.set(turnController.calculate(getTurnDegrees_180(), degrees));
    }

    /** Meters */
    private double getDrivePosition() {
        return driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    /** Meters per second */
    private double getDriveVelocity() {
        return driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    /** Degrees */
    private double getTurnDegrees_360() {
        return turningEncoder.getPosition() % 360;
    }

    private double getTurnDegrees_180() {
        double angle = getAbsolutePosition();
        if(Math.abs(angle) <= 180) {
            return angle * Math.signum(angle);
        } else {
            return Math.abs(angle) - 360 * Math.signum(angle);
        }
    }

    /** Degrees per second */
    private double getTurnVelocity() {
        return turningEncoder.getVelocity() * ModuleConstants.kTurningEncoderRPM2DegPerSec;
    }

    /** Degrees */
    private double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0; // 360
    }

    /** Degrees per second */
    private double getAbsoluteVelocity() {
        return absoluteEncoder.getVelocity().getValueAsDouble() * 360.0 / 60.0;
    }
}