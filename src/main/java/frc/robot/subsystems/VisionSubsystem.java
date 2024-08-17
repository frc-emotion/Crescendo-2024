package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera[] poseCameras;
    private PhotonPoseEstimator[] estimators;

    private PhotonCamera odCamera;

    public VisionSubsystem(String[] cameraIDs) {
        poseCameras = new PhotonCamera[cameraIDs.length];
        estimators = new PhotonPoseEstimator[cameraIDs.length];

        for(int i = 0; i < poseCameras.length; i++) {
            poseCameras[i] = new PhotonCamera(cameraIDs[i]);
            poseCameras[i].setPipelineIndex(VisionConstants.APRILTAG_PIPELINE_INDEX);
            
            estimators[i] = new PhotonPoseEstimator(VisionConstants.m_AprilTagFieldLayout, VisionConstants.POSE_STRATEGY, poseCameras[i], VisionConstants.ROBOT_TO_CAMS[i]);
        }

        odCamera = new PhotonCamera(VisionConstants.OD_CAM_ID);
        odCamera.setPipelineIndex(VisionConstants.OD_PIPELINE_INDEX);
    }

    public Optional<Pose2d> getEstimatedRobotPose(Pose2d lastPose) {
        Optional<Pose2d> avgPose = Optional.empty();
        for(int i = 0; i < estimators.length; i++) {
            estimators[i].setLastPose(lastPose);

            var estPose = estimators[i].update();

            if(avgPose.isPresent() && estPose.isPresent()) {
                avgPose = Optional.of(getAveragePose(avgPose.get(),estPose.get().estimatedPose.toPose2d()));
            } else if(estPose.isPresent()) {
                avgPose = Optional.of(estPose.get().estimatedPose.toPose2d());
            }
        }

        return avgPose;
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
}