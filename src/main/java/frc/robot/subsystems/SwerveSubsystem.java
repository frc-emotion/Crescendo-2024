package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModuleNeo[] swerveModules;

    private final AHRS gyro;

    private PIDController angleController;

    private GenericEntry kPEntry, kIEntry, kDEntry;
    

    public SwerveSubsystem() {
        swerveModules = new SwerveModuleNeo[] {
            new SwerveModuleNeo(
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            ),
            new SwerveModuleNeo(
                DriveConstants.kFrontRightDriveMotorPort,
                DriveConstants.kFrontRightTurningMotorPort,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            ),
            new SwerveModuleNeo(
                DriveConstants.kBackLeftDriveMotorPort,
                DriveConstants.kBackLeftTurningMotorPort,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
            ),
            new SwerveModuleNeo(
                DriveConstants.kBackRightDriveMotorPort,
                DriveConstants.kBackRightTurningMotorPort,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed
            )
        };

        gyro = new AHRS();
        // Robot.addPeriodicFunction(() -> calibrateModules(), 50);

        angleController = new PIDController(
            DriveConstants.kPThetaController,
            DriveConstants.kIThetaController,
            DriveConstants.kDThetaController
        );
        angleController.enableContinuousInput(0, 360);

        initShuffleboard();
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2d());
        driveRobotRelative(fieldSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        var moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for(int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(moduleStates[i], true);
        }
    }

    public double calculateThetaPID(Rotation2d angle) {
        return angleController.calculate(getRotation2d().getDegrees(), angle.getDegrees());
    }

    public void turnToAngle(Rotation2d angle) { 
        driveFieldRelative(new ChassisSpeeds(0, 0, calculateThetaPID(angle)));
    }

    public void calibrateModules() {
        for(SwerveModuleNeo moduleNeo : swerveModules) {
            moduleNeo.resetEncoders();
        }
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public boolean thetaPIDAtSetpoint() {
        return angleController.atSetpoint();
    }

    public boolean isGyroCalibrating() {
        return gyro.isCalibrating();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void updatePID() {
        angleController.setP(kPEntry.getDouble(DriveConstants.kPThetaController));
        angleController.setI(kIEntry.getDouble(DriveConstants.kIThetaController));
        angleController.setD(kDEntry.getDouble(DriveConstants.kDThetaController));
    }

    private void initShuffleboard() {
        if (!Constants.DEBUG_MODE_ACTIVE)
            return;

        ShuffleboardLayout frontLeftData, frontRightData, backLeftData, backRightData, sensorData;

        ShuffleboardTab moduleData = TabManager
                .getInstance()
                .accessTab(SubsystemTab.DRIVETRAIN);

        frontLeftData = moduleData.getLayout("Front Left", BuiltInLayouts.kList);
        frontRightData = moduleData.getLayout("Front Right", BuiltInLayouts.kList);
        backLeftData = moduleData.getLayout("Back Left", BuiltInLayouts.kList);
        backRightData = moduleData.getLayout("Back Right", BuiltInLayouts.kList);

        fillList(swerveModules[0], frontLeftData);
        fillList(swerveModules[1], frontRightData);
        fillList(swerveModules[2], backLeftData);
        fillList(swerveModules[3], backRightData);

        this.kPEntry = moduleData.add("kP", DriveConstants.kPThetaController).getEntry();

        this.kIEntry = moduleData.add("kI", DriveConstants.kIThetaController).getEntry();

        this.kDEntry = moduleData.add("kD", DriveConstants.kDThetaController).getEntry();

        sensorData = moduleData.getLayout("Gyro Data", BuiltInLayouts.kList);
        sensorData.addNumber("Gyro Heading", () -> getRotation2d().getDegrees());

    }

    private void fillList(SwerveModuleNeo module, ShuffleboardLayout layout) {
        layout.addNumber(
                "Absolute Position",
                () -> module.getAbsolutePostion());
        layout.addNumber(
                "Integrated Position",
                () -> module.getTurningPosition().getDegrees());
        layout.addNumber("Velocity", () -> module.getDriveVelocity());
        layout.withSize(2, 4);
    }
}
