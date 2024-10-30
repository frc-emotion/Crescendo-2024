package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera[] poseCameras;
    private PhotonPoseEstimator[] estimators;

    private PhotonCamera odCamera;
    private SwerveSubsystem swerveSubsystem;

    private SwerveDrivePoseEstimator poseEstimator;

    public VisionSubsystem(String[] cameraIDs, SwerveSubsystem swerveSubsystem) {
        poseCameras = new PhotonCamera[cameraIDs.length];
        estimators = new PhotonPoseEstimator[cameraIDs.length];
        this.swerveSubsystem = swerveSubsystem;
        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), new Pose2d());

        for(int i = 0; i < poseCameras.length; i++) {
            poseCameras[i] = new PhotonCamera(cameraIDs[i]);
            poseCameras[i].setPipelineIndex(VisionConstants.APRILTAG_PIPELINE_INDEX);
            
            estimators[i] = new PhotonPoseEstimator(VisionConstants.m_AprilTagFieldLayout, VisionConstants.POSE_STRATEGY, poseCameras[i], VisionConstants.ROBOT_TO_CAMS[i]);
        }

        // odCamera = new PhotonCamera(VisionConstants.OD_CAM_ID);
        // odCamera.setPipelineIndex(VisionConstants.OD_PIPELINE_INDEX);

        if(odCamera == null) DriverStation.reportWarning("Object Detection Disabled", false);
    }

    @Override
    public void periodic() {
        Pose2d lastPose = poseEstimator.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions());
        
        for(EstimatedRobotPose pose : getEstimatedRobotPose(lastPose)) {
            poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
    }

    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), pose);
    }

    /**
     * Retrieves the poses from the Photonvision pose estimators.
     *  
     * @param lastPose The last known robot pose
     * @return The list of robot poses
     */
    public List<EstimatedRobotPose> getEstimatedRobotPose(Pose2d lastPose) {
        var poses = new ArrayList<EstimatedRobotPose>(estimators.length);
        
        for(int i = 0; i < estimators.length; i++) {
            estimators[i].setLastPose(lastPose);

            var estPose = estimators[i].update();

            if(estPose.isPresent()) {
                poses.add(estPose.get());
            }
        }

        return poses;
    }

    public Optional<Translation2d> getRobotToObject() {
        var optCamToTarget = odCamera.getLatestResult();

        if(optCamToTarget.hasTargets()) {
            return Optional.of(optCamToTarget.targets.get(0).getBestCameraToTarget().getTranslation().toTranslation2d());
        } else {
            return Optional.empty();
        }
    }

    private Pose2d getAveragePose(Pose2d lastAverage, Pose2d estPose) {
        double x = (lastAverage.getX() + estPose.getX()) / 2.0;
        double y = (lastAverage.getY() + estPose.getY()) / 2.0;
        double rx = (lastAverage.getRotation().getDegrees() + estPose.getRotation().getDegrees()) / 2.0;
        
        return new Pose2d(x, y, Rotation2d.fromDegrees(rx));
    }

    public double getDistanceTo(Translation2d translation2d) {
        return getRobotPose().getTranslation().getDistance(translation2d);
    }
    
    public Rotation2d getAngleTo(Translation2d translation2d) {
        return PhotonUtils.getYawToPose(getRobotPose(), new Pose2d(translation2d, new Rotation2d()));
    }

    private Translation2d getRelativeTranslation(Translation2d translationOne, Translation2d translationTwo) {
        return new Translation2d(
            Math.abs(translationOne.getX() - translationTwo.getX()),
            Math.abs(translationOne.getY() - translationTwo.getY())
        );
    }
}