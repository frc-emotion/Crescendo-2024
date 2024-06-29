package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;

public class ObjectDetectionVision {
    public PhotonCamera camera;

    public ObjectDetectionVision(String cameraNTLabel) {
        camera = new PhotonCamera(cameraNTLabel);
        camera.setPipelineIndex(VisionConstants.OD_PIPELINE_INDEX);
    }

    public Optional<Transform2d> getObjectToRobotTransform() {
        return getObjectToRobotTransform(0);
    }

    /* TODO: Check if this method is correct */
    public Optional<Transform2d> getObjectToRobotTransform(int index) {
        var result = camera.getLatestResult();
        if(result.hasTargets()) {
            Transform3d camToObject3d = result.targets.get(index).getBestCameraToTarget();
            Transform3d robotToObject3d = camToObject3d.plus(VisionConstants.OD_CAM_TRANFORM.inverse());
            
            Transform2d transform2d = new Transform2d(
                robotToObject3d.getX(),
                robotToObject3d.getY(),
                new Rotation2d()
            );
            return Optional.of(transform2d);
        } else {
            return Optional.empty();
        }   
    }

    public boolean isObjectDetected() {
        return getObjectToRobotTransform().isPresent();
    }

    public static Pose2d getPickupPose(Translation2d objectTranslation, Pose2d currentPose) {
        return new Pose2d(
            objectTranslation,
            PhotonUtils.getYawToPose(currentPose, new Pose2d(objectTranslation, new Rotation2d()))
        );
    }
}
