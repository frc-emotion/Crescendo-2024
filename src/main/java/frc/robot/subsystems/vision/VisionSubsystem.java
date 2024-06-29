package frc.robot.subsystems.vision;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final AprilTagVision aprilTagVision;
    private final ObjectDetectionVision objectDetectionVision;

    private final SwerveDrivePoseEstimator swervePoseEstimator;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        aprilTagVision = new AprilTagVision(VisionConstants.APRILTAG_CAMERA_NT_LABELS);
        objectDetectionVision = new ObjectDetectionVision(VisionConstants.OD_CAMERA_NT_LABEL);

        swervePoseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            swerveSubsystem.getRotation2d(),
            swerveSubsystem.getModulePositions(),
            new Pose2d()
        );
    }

    @Override
    public void periodic() {
        aprilTagVision.updateEstimator(swervePoseEstimator);
    }

    public Pose2d getRobotPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public Optional<Pose2d> getObjectPose() {
        if(isObjectDetected()) {
            return Optional.of(
                getRobotPose().plus(objectDetectionVision.getObjectToRobotTransform().get())
            );
        } else {
            return Optional.empty();
        }
    }

    public boolean isObjectDetected() {
        return objectDetectionVision.isObjectDetected();
    }

    public double getDistanceTo(Pose2d pose) {
        Pose2d robotPose = getRobotPose();

        return Math.sqrt(
            Math.pow(robotPose.getX() - pose.getX(), 2) + Math.pow(robotPose.getY() - pose.getY(), 2)
        );
    }
}
