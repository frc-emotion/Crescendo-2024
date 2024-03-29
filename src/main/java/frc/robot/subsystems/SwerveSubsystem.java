package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;

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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Main Swerve Subsytem class
 * Holds gyro and odometry methods
 */
public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModuleNeo frontLeft = new SwerveModuleNeo(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModuleNeo frontRight = new SwerveModuleNeo(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModuleNeo backLeft = new SwerveModuleNeo(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModuleNeo backRight = new SwerveModuleNeo(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private double toDivideBy;
    private double driveSpeedMPS, driveAngularSpeedRPS;
    //private double driveAccelMPSS, driveAngularAccelRPSS;

    private ChassisSpeeds robotSpeeds;

    private ShuffleboardLayout frontLeftData, frontRightData, backLeftData, backRightData, sensorData;

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("PersianSwerveState", SwerveModuleState.struct).publish();

    private StructPublisher<Rotation2d> publisher2 = NetworkTableInstance.getDefault()
            .getStructTopic("PersianRotation", Rotation2d.struct).publish();

    private final PIDController autoThetaController, teleopThetaController;

    private GenericEntry kIEntry, kDEntry, kPEntry;

    public SwerveSubsystem() {

        // PIDController xController = new PIDController(AutoConstants.kPXController, 0,
        // 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0,
        // 0);

        autoThetaController = new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
                AutoConstants.kDThetaController);
        autoThetaController.enableContinuousInput(-180, 180);

        teleopThetaController = new PIDController(
            DriveConstants.kPThetaController, 
            DriveConstants.kIThetaController, 
            DriveConstants.kDThetaController
            );
        
        teleopThetaController.enableContinuousInput(-180, 180);
        teleopThetaController.setTolerance(20, 180);

        // new Thread() {
        // @Override
        // public void run() {
        // try {
        // sleep(1000);
        // zeroHeading();
        // } catch (InterruptedException e) {}
        // }
        // }.start();

        initShuffleboard();

        toDivideBy = OIConstants.kSpeedDivideAdjustment;
        driveSpeedMPS = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / toDivideBy;
        //driveAccelMPSS = DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / toDivideBy;
        driveAngularSpeedRPS = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / toDivideBy;
        //driveAngularAccelRPSS = DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / toDivideBy;

        robotSpeeds = new ChassisSpeeds();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception io) {
            }
        }).start();

    }

    public void updatePID() {
        this.teleopThetaController.setI(this.kIEntry.getDouble(ShooterConstants.kI));
        this.teleopThetaController.setD(this.kDEntry.getDouble(ShooterConstants.kD));
        this.teleopThetaController.setP(this.kPEntry.getDouble(ShooterConstants.kP));
    }

    public void setRawDriveSpeed(double speed) {
        frontRight.setRawDriveSpeed(speed);
        backRight.setRawDriveSpeed(speed);
        frontLeft.setRawDriveSpeed(speed);
        backLeft.setRawDriveSpeed(speed);
    }

    public void offsetGyro(double reading) {
        gyro.setAngleAdjustment(reading);
    }

    public double[] getSpeedType() {
        double[] toReturn = new double[2];
        toReturn[0] = driveSpeedMPS;
        toReturn[1] = driveAngularSpeedRPS;
        return toReturn;
    }

    public void setMaxSpeeds(double driveSpeedMPS, double driveAccelMPSS, double driveAngularSpeedRPS,
            double driveAngularAccelRPSS) {
        this.driveSpeedMPS = driveSpeedMPS;
        //this.driveAccelMPSS = driveAccelMPSS;
        this.driveAngularSpeedRPS = driveAngularSpeedRPS;
        //this.driveAngularAccelRPSS = driveAngularAccelRPSS;
    }

    public void autoGyro() {
        gyro.setAngleAdjustment(180);
    }

    public double getPitch() {
        return Units.degreesToRadians((gyro.getPitch()));
    }

    public double getRoll() {
        return Units.degreesToRadians((gyro.getRoll()));
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public boolean isGyroCalibrating() {
        return gyro.isCalibrating();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
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
        return robotSpeeds;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        robotSpeeds = speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speedGiven) {
        setChassisSpeeds(speedGiven);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                speedGiven);
        setModuleStates(moduleStates);
    }

    public void driveFieldRelative(ChassisSpeeds speedGiven) {
        setChassisSpeeds(speedGiven);

        speedGiven = ChassisSpeeds.fromFieldRelativeSpeeds(speedGiven, getRotation2d());

        setModuleStates(
                DriveConstants.kDriveKinematics.toSwerveModuleStates(speedGiven));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    public void periodic() {

        // SmartDashboard.putNumber("Gyro Reading", getHeading());
        // SmartDashboard.putNumber("Gyro Pitch", getPitch());
        // SmartDashboard.putNumber("Gyro Roll", getRoll());

        SwerveModuleState[] moduleStates = getModuleStates();
        Rotation2d currentRotation = getRotation2d();

        publisher.set(moduleStates);
        publisher2.set(currentRotation);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], false);
        frontRight.setDesiredState(desiredStates[1], false);
        backLeft.setDesiredState(desiredStates[2], false);
        backRight.setDesiredState(desiredStates[3], false);
    }

    public void setModuleStates(
            SwerveModuleState[] desiredStates,
            boolean station) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], station);
        frontRight.setDesiredState(desiredStates[1], station);
        backLeft.setDesiredState(desiredStates[2], station);
        backRight.setDesiredState(desiredStates[3], station);
    }

    private void initShuffleboard() {
        if (!Constants.DEBUG_MODE_ACTIVE)
            return;

        ShuffleboardTab moduleData = TabManager
                .getInstance()
                .accessTab(SubsystemTab.DRIVETRAIN);
        frontLeftData = moduleData.getLayout("Front Left", BuiltInLayouts.kList);
        frontRightData = moduleData.getLayout("Front Right", BuiltInLayouts.kList);
        backLeftData = moduleData.getLayout("Back Left", BuiltInLayouts.kList);
        backRightData = moduleData.getLayout("Back Right", BuiltInLayouts.kList);
        fillList(frontLeft, frontLeftData);
        fillList(frontRight, frontRightData);
        fillList(backLeft, backLeftData);
        fillList(backRight, backRightData);

        this.kPEntry = moduleData.add("kP", DriveConstants.kPThetaController).getEntry();

        this.kIEntry = moduleData.add("kI", DriveConstants.kIThetaController).getEntry();

        this.kDEntry = moduleData.add("kD", DriveConstants.kDThetaController).getEntry();

        sensorData = moduleData.getLayout("Gyro Data", BuiltInLayouts.kList);
        sensorData.addNumber("Gyro Heading", () -> getHeading());
        sensorData.addNumber("Gyro Pitch", () -> getPitch());
        sensorData.addNumber("Gyro Roll", () -> getRoll());

    }

    private void fillList(SwerveModuleNeo module, ShuffleboardLayout layout) {
        layout.addNumber(
                "Absolute Position",
                () -> module.getAbsolutePostion());
        layout.addNumber(
                "Integrated Position",
                () -> module.getTurningPosition());
        layout.addNumber("Velocity", () -> module.getDriveVelocity());
        layout.withSize(2, 4);
    }

    public double calculateThetaPID(double measurement, double setpoint, boolean auto) {
        if (auto) {
            return autoThetaController.calculate(measurement, setpoint);
        }
        return teleopThetaController.calculate(measurement, setpoint);
 
        
    }

    public boolean thetaPIDAtSetpoint(boolean auto) {
        if (auto) {
            return autoThetaController.atSetpoint();
        }
        return teleopThetaController.atSetpoint();
    }

}
