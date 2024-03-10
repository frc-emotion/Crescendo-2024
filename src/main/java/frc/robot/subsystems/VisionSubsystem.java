package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.RectanglePoseArea;
import frc.robot.util.TabManager;
import frc.robot.util.VisionTypes;
import frc.robot.util.TabManager.SubsystemTab;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSubsystem extends SubsystemBase {

    private final Field2d m_field = new Field2d();
    private final Field2d m_field2 = new Field2d();

    private SendableChooser<VisionTypes> visionType = new SendableChooser<>();

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("PersianPoseOdometryVision", Pose2d.struct).publish();

    private StructPublisher<Pose2d> posePublisher2 = NetworkTableInstance.getDefault()
            .getStructTopic("PersianPoseOdometryOnly", Pose2d.struct).publish();

    private SwerveSubsystem swerveSubsystem;

    private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0),
            new Translation2d(16.541, 8.211));

    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            new Rotation2d(),
            swerveSubsystem.getModulePositions(),
            new Pose2d());

    public final SwerveDriveOdometry poseOdometryEstimator = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            new Rotation2d(),
            swerveSubsystem.getModulePositions(),
            new Pose2d());

    private VisionTypes methodToUse = VisionTypes.NO_VISION;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;

        visionType.setDefaultOption("NO VISION (why would you want this?)", VisionTypes.NO_VISION);
        visionType.addOption("Torpedo Vision", VisionTypes.TORPEDO);
        visionType.addOption("Preset STD", VisionTypes.PRESET_STD);
        visionType.addOption("Custom STD", VisionTypes.CUSTOM_STD);
        visionType.addOption("Calculations Based Vision", VisionTypes.CRAZY_MATH);

        initShuffleboard();
    }


    public void updateEstimator() {
        poseEstimator.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions());
        poseOdometryEstimator.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions()); // testing
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // testing
    public Pose2d getCurrentOdoPose() {
        return poseOdometryEstimator.getPoseMeters();
    }

    public PoseEstimate getVisionPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    // testing
    public void resetOdometry(Pose2d pose) {
        poseOdometryEstimator.resetPosition(
                swerveSubsystem.getRotation2d(),
                swerveSubsystem.getModulePositions(),
                pose);
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(
                swerveSubsystem.getRotation2d(),
                swerveSubsystem.getModulePositions(),
                pose);
    }

    public boolean tagDetected() {
        return LimelightHelpers.getTV("limelight");
    }

    public int getNumTags() {
        return getVisionPose().tagCount;
    }

    public double getAvgTagDist() {
        return getVisionPose().avgTagDist;
    }

    // VISION UPDATING - VERSIONS 1-4

    // LimelightLib-given constants for Std Devs
    public void updateVision1() {
        PoseEstimate poseEstimate = getVisionPose();

        if (poseEstimate.tagCount >= 2) {

            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999999)); // documentation defaults,
                                                                                             // change?
            poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }

    }

    // distant dependant formula for Std Devs
    public void updateVision2() {
        PoseEstimate poseEstimate = getVisionPose();

        if (tagDetected()) {
            poseEstimator.setVisionMeasurementStdDevs(getEstimationStdDevs(poseEstimate));
            poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(PoseEstimate estimatedPose) {
        int numTags = estimatedPose.tagCount;
        double avgDist = estimatedPose.avgTagDist;

        var estStdDevs = VecBuilder.fill(4, 4, 8); // kSingleTagDevs

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = VecBuilder.fill(0.5, 0.5, 1); // kMultiTagDevs
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        // Increase std devs based on (average) distance
        else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }

    // old torpedo code that worked
    public void updateVision3() {
        LimelightHelpers.Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;

        if (!(results.botpose[0] == 0 && results.botpose[1] == 1) && tagDetected()) {
            // Error saying that was not visible so i made the method toPose2D public, not
            // sure if that breaks anything or matters
            Pose2d estPose = LimelightHelpers.toPose2D(results.botpose_wpiblue);
            poseEstimator.addVisionMeasurement(estPose,
                    Timer.getFPGATimestamp() - (results.latency_capture / 1000.0) - (results.latency_pipeline / 1000.0),
                    VecBuilder.fill(4, 4, 8));
        }

    }

    // Team 6391-inspired conditionals to set Std Dev scenarios
    public void updateVision4() {
        double confidence = 0;
        // If we don't update confidence then we don't send the measurement
        // this number is used as the x and y Std Dev

        boolean trust = false;

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        int tagCount = limelightMeasurement.tagCount;
        double tagDist = limelightMeasurement.avgTagDist;
        Pose2d pose = limelightMeasurement.pose;

        // No tag found so check no further or pose not within field boundary
        if (tagCount >= 1 && fieldBoundary.isPoseWithinArea(pose)) {
            // Excluding different measurements that are absolute showstoppers even with
            // full trust
            if (tagDist < Constants.VisionConstants.TAG_DETECTION_THRESHOLD
                    && swerveSubsystem.getChassisSpeeds().omegaRadiansPerSecond < Math.PI) {
                // Reasons to blindly trust as much as odometry
                if (trust || DriverStation.isDisabled() ||
                        (tagCount >= 2 && tagDist < Units.feetToMeters(10))) {
                    confidence = 0.2;
                    trust = false;
                } else {
                    // High trust level anything less than this we shouldn't bother with
                    double compareDistance = pose.getTranslation().getDistance(getCurrentPose().getTranslation());
                    if (compareDistance < 0.5 ||
                            (tagCount >= 2 && tagDist < Units.feetToMeters(20)) ||
                            (tagCount == 1 && tagDist < Units.feetToMeters(10))) {
                        double tagDistance = Units.metersToFeet(tagDist);
                        // Double the distance for solo tag
                        if (tagCount == 1) {
                            tagDistance = tagDistance * 2;
                        }
                        // Add up to .2 confidence depending on how far away
                        confidence = 0.7 + (tagDistance / 10);
                    }
                }
            }
        }

        if (confidence > 0) {
            SmartDashboard.putBoolean("PoseUpdate", true);
            SmartDashboard.putNumber("LLConfidence", confidence);
            poseEstimator.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds,
                    VecBuilder.fill(confidence, confidence, 99));
        } else {
            SmartDashboard.putBoolean("PoseUpdate", false);
        }
        updateField();
    }


    // SHUFFLEBOARD

    public void updateField() {
        m_field.setRobotPose(getCurrentPose());
        m_field2.setRobotPose(getCurrentOdoPose());
    }

    public void setVisionType(VisionTypes type) {
        methodToUse = type;
    }

    public VisionTypes getVisionType() {
        return methodToUse;
    }

    public VisionTypes getSelectedType() {
        return visionType.getSelected();
    }

    public void snapOdometry() {
        resetOdometry(getCurrentPose());
    }

    public void updateNetworkTable() {
        posePublisher.set(poseEstimator.getEstimatedPosition());
        posePublisher2.set(poseOdometryEstimator.getPoseMeters());
    }

    private void initShuffleboard() {
        if (!Constants.VisionConstants.VISION_DEBUG_MODE)
            return;

        ShuffleboardTab visionData = TabManager.getInstance().accessTab(SubsystemTab.VISION);

        visionData.add("Odometry + Vision Field", m_field).withWidget(BuiltInWidgets.kField);
        visionData.add("Odometry Only Field", m_field2).withWidget(BuiltInWidgets.kField);
        visionData.addNumber("Num Tags", () -> getNumTags());
        visionData.addNumber("Tag Dist", () -> getAvgTagDist());
        visionData.add("Snap Odometry to Vision+Odometry", new InstantCommand(() -> snapOdometry()));
        visionData.addString("Current Mode", () -> getVisionType().toString());
    }
  
}