package frc.robot.subsystems.swerve;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Time;
// import edu.wpi.first.units.Voltage;
// import java.util.function.Consumer;
// import edu.wpi.first.networktables.StringPublisher;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.SwerveLimiter.LimiterConstraints;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Main Swerve Subsytem class
 */
public class SwerveSubsystem extends SubsystemBase {
    SendableNumber Module_kP = new SendableNumber(SubsystemTab.DRIVETRAIN, "Drive kP", DriveConstants.MODULE_kP);
    SendableNumber Module_kI = new SendableNumber(SubsystemTab.DRIVETRAIN, "Drive kI", DriveConstants.MODULE_kI);
    SendableNumber Module_kD = new SendableNumber(SubsystemTab.DRIVETRAIN, "Drive kD", DriveConstants.MODULE_kD);
    SendableNumber MaxSpeed = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Drive Speed", DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    SendableNumber MaxAccel = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Drive Accel", DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    SendableNumber MaxAngSpd = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Angular Speed", DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
    SendableNumber MaxAngAccel = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Angular Accel", DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private final SwerveIO io;
    private static final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    PIDController autoThetaController, teleopThetaController;

    private ChassisSpeeds robotSpeeds;

    private ShuffleboardLayout frontLeftData, frontRightData, backLeftData, backRightData, sensorData;

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("PersianSwerveState", SwerveModuleState.struct).publish();

    private StructPublisher<Rotation2d> publisher2 = NetworkTableInstance.getDefault()
            .getStructTopic("PersianRotation", Rotation2d.struct).publish();

    private SwerveDrivePoseEstimator poseEstimator;

    // private StringPublisher publisher3 =
    // NetworkTableInstance.getDefault().getStringTopic("SysIdData").publish();

    // private SysIdRoutine sysIdRoutine;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;

        // this.sysIdRoutine = new SysIdRoutine(
        // new SysIdRoutine.Config(Volts.of(0.5).per(Seconds.of(1.0)), Volts.of(5),
        // Seconds.of(5), (state) -> publisher3.set(state.toString())),
        // new SysIdRoutine.Mechanism(
        // (volts) -> this.setVoltage(volts.in(Volts)),
        // null, // No log consumer, since data is recorded by URCL
        // this
        // )
        // );

        // The methods below return Command objects
        // sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        // sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
        // sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        // sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception ex) {
            }
        }).start();

        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), new Pose2d());

        initShuffleboard();
    }

    // public Command quasistaticCommand(boolean forward) {
    // if (forward) {
    // return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }
    // return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);

    // }

    // public Command dynamicCommand(boolean forward) {
    // if (forward) {
    // return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    // }
    // return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    // }

    public void updatePID() {
       io.updateConstants(Module_kP.get(), Module_kI.get(), Module_kD.get());
    }

    public void offsetGyro(double offset) {
        io.setGyroOffset(offset);
    }

    public void updateVisionPose(EstimatedRobotPose estimatedVisionPose) {
        poseEstimator.addVisionMeasurement(estimatedVisionPose.estimatedPose.toPose2d(), estimatedVisionPose.timestampSeconds);
    }

    public void setMaxDriveConstraints(LimiterConstraints constraints) {
        io.updateDriveConstraints(constraints);
    }

    public void setMaxDriveConstraints(double maxSpeed, double maxAccel, double maxAngSpeed, double maxAngAccel) {
        setMaxDriveConstraints(new LimiterConstraints(maxSpeed, maxAccel, maxAngSpeed, maxAngAccel));
    }

    public void updateDriveConstraints() {
        setMaxDriveConstraints(new LimiterConstraints(MaxSpeed.get(), MaxAccel.get(), MaxAngSpd.get(), MaxAngAccel.get()));
    }

    public void autoGyro() {
        offsetGyro(180);
    }

    public double getPitch() {
        return Units.degreesToRadians(inputs.pitch);
    }

    public double getRoll() {
        return Units.degreesToRadians(inputs.roll);
    }

    public void zeroHeading() {
        io.resetGyro();
    }

    public boolean isGyroCalibrating() {
        return inputs.isGyroCalibrating;
    }

    public double getHeading() {
        return inputs.yaw;
    }

    public double getHeading_180() {
        if (getHeading() > 180) {
            return (getHeading() - 180) * -1;
        } else {
            return getHeading();
        }
    }

    public Rotation2d getRotation2d() {
        // return gyro.getRotation2d();
        return Rotation2d.fromDegrees(getHeading());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return inputs.robotSpeeds;
    }

    public void driveRobotRelative(ChassisSpeeds speedGiven) {
        io.driveRobotRelative(speedGiven);
    }

    public void driveFieldRelative(ChassisSpeeds speedGiven) {
        io.driveFieldRelative(speedGiven);
        
    }

    public SwerveModulePosition[] getModulePositions() {
        return inputs.modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        return inputs.moduleStates;
    }

    @Override
    public void periodic() {

        // SmartDashboard.putNumber("Gyro Reading", getHeading());
        // SmartDashboard.putNumber("Gyro Pitch", getPitch());
        // SmartDashboard.putNumber("Gyro Roll", getRoll());

        poseEstimator.update(getRotation2d(), getModulePositions());

        SwerveModuleState[] moduleStates = getModuleStates();
        Rotation2d currentRotation = getRotation2d();

        publisher.set(moduleStates);
        publisher2.set(currentRotation);
    }

    public void stopModules() {
        io.stop();
    }

    private void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == Constants.RobotDataMode.MATCH)
            return;

        ShuffleboardTab moduleData = TabManager
                .getInstance()
                .accessTab(SubsystemTab.DRIVETRAIN);

        // moduleData.add("Quasistatic Forward", quasistaticCommand(true));
        // moduleData.add("Quasistatic Reverse", quasistaticCommand(false));
        // moduleData.add("Dynamic Forward", dynamicCommand(true));
        // moduleData.add("Dynamic Reverse", dynamicCommand(false));

        frontLeftData = moduleData.getLayout("Front Left", BuiltInLayouts.kList);
        frontRightData = moduleData.getLayout("Front Right", BuiltInLayouts.kList);
        backLeftData = moduleData.getLayout("Back Left", BuiltInLayouts.kList);
        backRightData = moduleData.getLayout("Back Right", BuiltInLayouts.kList);
        if(io instanceof SwerveIOReal) {
            ((SwerveIOReal) io).fillList(0, frontLeftData);
            ((SwerveIOReal) io).fillList(1, frontRightData);
            ((SwerveIOReal) io).fillList(2, backLeftData);
            ((SwerveIOReal) io).fillList(3, backRightData);
        }

        sensorData = moduleData.getLayout("Gyro Data", BuiltInLayouts.kList);
        sensorData.addNumber("Gyro Heading", () -> getHeading());
        sensorData.addNumber("Gyro Pitch", () -> getPitch());
        sensorData.addNumber("Gyro Roll", () -> getRoll());

    }

    public double calculateThetaPID(double measurement, double setpoint, boolean auto) {
        // Remove clamping if not working
        if (auto) {
            return autoThetaController.calculate(-measurement, 360 - setpoint);
        }
        // return MathUtil.clamp(teleopThetaController.calculate(-measurement, 360 - setpoint), -540, 540);
        return teleopThetaController.calculate(-measurement, setpoint);

    }

    public boolean thetaPIDAtSetpoint(boolean auto) {
        if (auto) {
            return autoThetaController.atSetpoint();
        }
        return teleopThetaController.atSetpoint();
    }

}
