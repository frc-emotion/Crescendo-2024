package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera orangePiCamera, limelightCamera;

    private PhotonPoseEstimator visionPoseEstimator;

    private PhotonPipelineResult limelightResults;

    public VisionSubsystem() {
        orangePiCamera = new PhotonCamera("arducam");
        limelightCamera = new PhotonCamera("limelight");

        orangePiCamera.setPipelineIndex(VisionConstants.APRILTAG_PIPELINE_INDEX);
        limelightCamera.setPipelineIndex(VisionConstants.OD_PIPELINE_INDEX);
        
        visionPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.m_AprilTagFieldLayout,
            VisionConstants.POSE_STRATEGY,
            VisionConstants.ROBOT_TO_PI_CAM
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d lastPose) {
        visionPoseEstimator.setReferencePose(lastPose);
        return visionPoseEstimator.update();
    }
}