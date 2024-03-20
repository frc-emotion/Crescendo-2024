package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
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
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
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

    public final SwerveDrivePoseEstimator poseEstimator;

    public final SwerveDriveOdometry poseOdometryEstimator;

    private VisionTypes methodToUse = VisionTypes.NO_VISION;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;

        visionType.setDefaultOption("NO VISION (why would you want this?)", VisionTypes.NO_VISION);
        // visionType.addOption("Torpedo Vision", VisionTypes.TORPEDO);
        // visionType.addOption("Preset STD", VisionTypes.PRESET_STD);
        visionType.addOption("Custom STD", VisionTypes.CUSTOM_STD);
        // visionType.addOption("Calculations Based Vision", VisionTypes.CRAZY_MATH);
        visionType.addOption("Limelight Docs Vision", VisionTypes.LLDOCS);

        this.poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            new Rotation2d(),
            swerveSubsystem.getModulePositions(),
            new Pose2d());

        this.poseOdometryEstimator = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            new Rotation2d(),
            swerveSubsystem.getModulePositions(),
            new Pose2d());

        initShuffleboard();
    }

    public void updateOdometry() {
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

        resetOdometry(pose);
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

    public LimelightTarget_Fiducial[] getTagFiducial() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials;
    }

    public double[] getTagIDs() {
        var fiducials = getTagFiducial();
        double[] ids = new double[fiducials.length];
        for(int i = 0; i < ids.length; i++) {
            ids[i] = fiducials[i].fiducialID;
        }
        return ids;
    }

    public boolean isIdDetected(double targetID) {
        for(double id : getTagIDs()) {
            if(targetID == id) return true;
        }
        return false;
    }

    public double getDistanceTo(Translation2d location) {
        return getCurrentPose().getTranslation().getDistance(location);
    }

    /**
     * Finds the distance from the robot to a certain location on the field. Uses the robot odometry (no vision) to determine distance.
     * 
     * @param location  The location to check the distance to.
     * @return  The distance to the location.
     */
    public double getOdoDistanceTo(Translation2d location) {
        return getCurrentOdoPose().getTranslation().getDistance(location);
    }

    public double getTX() {
        return LimelightHelpers.getTX("limelight");
    }

    // VISION UPDATING - VERSIONS 1-4

    // LimelightLib-given constants for Std Devs
    // public void updateVision1() {
    //     PoseEstimate poseEstimate = getVisionPose();

    //     if (poseEstimate.tagCount >= 2) {

    //         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999999)); // documentation defaults,
    //                                                                                          // change?
    //         poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    //     }

    // }

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

        // var estStdDevs = VecBuilder.fill(4, 4, 8); // kSingleTagDevs
        var estStdDevs = VecBuilder.fill(5, 5, 9);

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            //estStdDevs = VecBuilder.fill(0.5, 0.5, 1); // kMultiTagDevs
            estStdDevs = VecBuilder.fill(1, 1, 2);
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        // Increase std devs based on (average) distance
        else {
            // estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            estStdDevs = estStdDevs.times((1 + (avgDist * avgDist / 30)) * 0.80);
        }
        return estStdDevs;
    }

    public void updateVision5() {
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        // invalid LL data
        if (botPose.pose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
                .getDistance(botPose.pose.getTranslation());
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
        if (getNumTags() > 0) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (getNumTags() >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (botPose.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (botPose.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            poseEstimator.addVisionMeasurement(botPose.pose,
                    Timer.getFPGATimestamp() - botPose.latency);
        }
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

    public void snapVisionOdo() {
        resetOdometry(getCurrentOdoPose());
    }

    public double getDifference() {
        return getCurrentPose().getTranslation().getDistance(getVisionPose().pose.getTranslation());
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
        visionData.addNumber("Difference btw Current Pose and New Vision Estimate", () -> getDifference());
        visionData.add("Snap Odometry to Vision+Odometry", new InstantCommand(() -> snapOdometry()));
        visionData.add("Snap Vision+Odometry to Odometry", new InstantCommand(() -> snapVisionOdo()));
        visionData.addString("Current Mode", () -> getVisionType().toString());
        visionData.add("Vision Method", visionType);
    }

}