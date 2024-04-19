package frc.robot.subsystems.other;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private SwerveSubsystem swerveSubsystem;
    private final PhotonCamera camera;

    private final PhotonPoseEstimator photonPoseEstimator;
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        camera = new PhotonCamera("photoncamera");
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.CAMERA_TRANSLATION);

        swervePoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), new Pose2d());
    }

    @Override
    public void periodic() {
        swervePoseEstimator.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions());
        if(hasTargets()) {
            var estPose = getVisionPose();
            if(estPose.isPresent()) {
                swervePoseEstimator.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);
            }
        }
    }

    public Optional<EstimatedRobotPose> getVisionPose() {
        return photonPoseEstimator.update();
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), pose);
    }


    public Optional<Double> getDistanceToTag() {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {
           var target = result.getBestTarget();
           var tagPose = getTagPose3d(target).get().toPose2d();

           return Optional.of(
                PhotonUtils.estimateCameraToTarget(
                    target.getBestCameraToTarget().getTranslation().toTranslation2d(), 
                    tagPose, 
                    swerveSubsystem.getRotation2d()
                ).getTranslation().getDistance(new Translation2d())
            );
        }
        return Optional.empty();
    }

    public Optional<Double> getDistanceToTag(int id) {
        var target = getTarget(id);

        if(target.isPresent()) {
            var tagPose = getTagPose3d(target.get()).get().toPose2d();

            return Optional.of(
                    PhotonUtils.estimateCameraToTarget(
                        target.get().getBestCameraToTarget().getTranslation().toTranslation2d(), 
                        tagPose, 
                        swerveSubsystem.getRotation2d()
                    ).getTranslation().getDistance(new Translation2d())
                );
        }
        return Optional.empty();
    }

    public Optional<Pose3d> getTagPose3d() {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {
            return getTagPose3d(result.getBestTarget());
        }
        return Optional.empty();
    }

    public Optional<Pose3d> getTagPose3d(int tagId) {
        var target = getTarget(tagId);
        
        if(target.isPresent()) {
            return getTagPose3d(target.get());
        }
        return Optional.empty();
    }

    private Optional<Pose3d> getTagPose3d(PhotonTrackedTarget target) {
        var robotPose = getVisionPose();
        if(robotPose.isPresent()) {
            var cameraPose = robotPose.get().estimatedPose.transformBy(VisionConstants.CAMERA_TRANSLATION);
            return Optional.of(cameraPose.transformBy(target.getBestCameraToTarget()));
        }
        return Optional.empty();
    }

    public Optional<Double> getVisionDistanceTo(Pose2d targetPose) {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {
            var currentPose = getVisionPose();

            if(currentPose.isPresent()) {
                return Optional.of(PhotonUtils.getDistanceToPose(currentPose.get().estimatedPose.toPose2d(), targetPose));
            }
        }
        return Optional.empty();
    }

    public double getDistanceTo(Pose2d pose) {
        return PhotonUtils.getDistanceToPose(getPose(), pose);
    }

    public Rotation2d getAngleTo(Pose2d pose) {
        return PhotonUtils.getYawToPose(getPose(), pose);
    }

    public Optional<Rotation2d> getVisionAngleTo(int tagId) {
        var target = getTarget(tagId);

        if(target.isPresent()) {
            return Optional.of(Rotation2d.fromDegrees(target.get().getYaw()));
        } else {
            return Optional.empty();
        }
    }

    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    public boolean hasTarget(int id) {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {
            var targets = result.getTargets();

            for(PhotonTrackedTarget target : targets) {
                if(target.getFiducialId() == id) return true;
            }
        }
        return false;
    }

    public Optional<PhotonTrackedTarget> getTarget(int id) {
       var result = camera.getLatestResult();

        if(result.hasTargets()) {
            var targets = result.getTargets();

            for(PhotonTrackedTarget target : targets) {
                if(target.getFiducialId() == id) return Optional.of(target);
            }
        }
        return Optional.empty();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {
            return Optional.of(result.getBestTarget());
        } else {
            return Optional.empty();
        }
    }

    public void takePicture() {
        camera.takeInputSnapshot();
        camera.takeOutputSnapshot();
    }
}
