package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.VisionConstants;

public class AprilTagVision {
    private PhotonCamera[] cameras;
    private PhotonPoseEstimator[] estimators;

    private ArrayList<Optional<EstimatedRobotPose>> estimatedPoses;
    
    public AprilTagVision(String... cameraNTLabels) {
        cameras = new PhotonCamera[cameraNTLabels.length];
        estimators = new PhotonPoseEstimator[cameraNTLabels.length];

        for(int i = 0; i < cameraNTLabels.length; i++) {
            cameras[i] = new PhotonCamera(cameraNTLabels[i]);
            cameras[i].setPipelineIndex(VisionConstants.APRILTAG_PIPELINE_INDEX);

            estimators[i] = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                cameras[i],
                VisionConstants.CAMERA_TRANSLATIONS[i]
            );
            estimatedPoses = new ArrayList<Optional<EstimatedRobotPose>>(cameras.length);
        }
    }

    public void periodic() {
        for(int i = 0; i < cameras.length; i++) {
            estimatedPoses.set(i, estimators[i].update());
        }
    }

    public ArrayList<Optional<EstimatedRobotPose>> getEstimatedPoses() {
        return estimatedPoses;
    }
}
