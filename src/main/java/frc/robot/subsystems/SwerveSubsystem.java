package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Main Swerve Subsytem class
 * Holds gyro and odometry methods
 */
public class SwerveSubsystem extends SubsystemBase {

    private static final PathConstraints kPathConstraints = new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
        AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
        AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared
    );

    private final SwerveModuleNeo frontLeft = new SwerveModuleNeo(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModuleNeo frontRight = new SwerveModuleNeo(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModuleNeo backLeft = new SwerveModuleNeo(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModuleNeo backRight = new SwerveModuleNeo(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private double toDivideBy;

    // private final PoseEstimator visionPoseEstimator = new PoseEstimator();

    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getRotation2d(), // Was empty pose 2d before
        getModulePositions(),
        new Pose2d()
    ); // FIX add the starting pose estimate?

    private ChassisSpeeds robotSpeeds;

    private ShuffleboardLayout frontLeftData;
    private ShuffleboardLayout frontRightData;
    private ShuffleboardLayout backLeftData;
    private ShuffleboardLayout backRightData;
    private Field2d m_field;

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("PersianSwerveState", SwerveModuleState.struct).publish();
    
    private StructPublisher<Rotation2d> publisher2 = NetworkTableInstance.getDefault()
        .getStructTopic("PersianRotation", Rotation2d.struct).publish();
    

    public SwerveSubsystem() {

        //PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        //PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        //PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        //thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception io) {}
        })
            .start();

        initShuffleboard();

        toDivideBy = 2.0;

        BooleanSupplier supp = () -> { return true; };

        AutoBuilder.configureHolonomic(
            this::getCurrentPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(AutoConstants.kMaxSpeedMetersPerSecond, DriveConstants.kWheelBase, new ReplanningConfig()),
            supp,
            this
        );

        Pathfinding.setPathfinder(new LocalADStar());
        
    }

    public double[] getSpeedType() {
        double[] toReturn = new double[2];
        toReturn[0] =
            Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond /
            toDivideBy;
        toReturn[1] =
            Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond /
            toDivideBy;
        return toReturn;
    }

    public void setSpeedType(double divisor) {
        toDivideBy = divisor;
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

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // Resets current pose to a specified pose.
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
            getRotation2d(),
            getModulePositions(),
            pose
        );
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
            speedGiven
        );
        setModuleStates(moduleStates);
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
        // Updates with drivetrain sensors
        poseEstimator.update(getRotation2d(), getModulePositions());

        // Pair<Pose2d, Double> result = visionPoseEstimator.getEstimatedPose();

        // Adds vision
        // poseEstimator.addVisionMeasurement(result.getFirst(), result.getSecond());

        // m_field.setRobotPose(getCurrentPose());
        SmartDashboard.putNumber("Gyro Reading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
        SmartDashboard.putNumber("Gyro Roll", getRoll());

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
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );
        frontLeft.setDesiredState(desiredStates[0], false);
        frontRight.setDesiredState(desiredStates[1], false);
        backLeft.setDesiredState(desiredStates[2], false);
        backRight.setDesiredState(desiredStates[3], false);
    }

    public void setModuleStates(
        SwerveModuleState[] desiredStates,
        boolean station
    ) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );
        frontLeft.setDesiredState(desiredStates[0], station);
        frontRight.setDesiredState(desiredStates[1], station);
        backLeft.setDesiredState(desiredStates[2], station);
        backRight.setDesiredState(desiredStates[3], station);
    }

    private void initShuffleboard() {
        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.DRIVETRAIN);
        frontLeftData =
            moduleData.getLayout("Front Left", BuiltInLayouts.kList);
        frontRightData =
            moduleData.getLayout("Front Right", BuiltInLayouts.kList);
        backLeftData = moduleData.getLayout("Back Left", BuiltInLayouts.kList);
        backRightData =
            moduleData.getLayout("Back Right", BuiltInLayouts.kList);
        fillList(frontLeft, frontLeftData);
        fillList(frontRight, frontRightData);
        fillList(backLeft, backLeftData);
        fillList(backRight, backRightData);

        m_field = new Field2d();

        TabManager
            .getInstance()
            .addFieldWidget(
                TabManager.getInstance().accessTab(SubsystemTab.AUTON),
                BuiltInWidgets.kField,
                "Pose",
                m_field,
                new int[] { 0, 0 },
                new int[] { 6, 4 }
            );
    }

    private void fillList(SwerveModuleNeo module, ShuffleboardLayout layout) {
        layout.addNumber(
            "Absolute Position",
            () -> module.getAbsolutePostion()
        );
        layout.addNumber(
            "Integrated Position",
            () -> module.getTurningPosition()
        );
        layout.addNumber("Velocity", () -> module.getDriveVelocity());
        layout.withSize(2, 4);
    }

    /**
     * Creates a new Command to pathfind to a certain pose using
     * only the target pose.
     * 
     * @param pose  The target pose
     * @return      The new command to reach the target pose
     */
    public Command navigateToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
            pose, 
           kPathConstraints
        );
    }

    /**
     * Creates a new Command to pathfind to a certain pose using
     * the target pose and end velocity.
     * 
     * @param pose          The target pose
     * @param endVelocity   The end velocity
     * @return              The new command to reach the target pose
     */
    public Command navigateToPose(Pose2d pose, double endVelocity) {
        return AutoBuilder.pathfindToPose(
            pose, 
           kPathConstraints,
           endVelocity
        );
    }

    /**
     * Creates a new Command to pathfind to a certain pose using
     * the target pose, end velocity, and the distance the robot
     * should travel before turning to the target pose's heading.
     * 
     * @param pose                  The target pose
     * @param endVelocity           The end velocity
     * @param rotationDelayDistance The distance the robot should travel before turning
     * @return                      The new Command to reach the target pose
     */
    public Command navigateToPose(Pose2d pose, double endVelocity, double rotationDelayDistance) {
        return AutoBuilder.pathfindToPose(
            pose, 
           kPathConstraints,
           endVelocity,
           rotationDelayDistance
        );
    }
}
