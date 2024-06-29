package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Adjusts the rotation of the robot and pivot in order to aim toward the
 * speaker.
 * Uses vision to aim toward the speaker AprilTag.
 */
public class VisionSpeakerSwerveCommand extends SnapSwerveCommand {
    private final VisionSubsystem visionSubsystem;
    private final PivotSubsystem pivotSubsystem;

    public VisionSpeakerSwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc,
            VisionSubsystem visionSubsystem,
            PivotSubsystem pivotSubsystem) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc, 0);
        this.visionSubsystem = visionSubsystem;
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void execute() {

        double pivotAngle = calculatePivotAngle();
        if (!pivotSubsystem.isAtTarget(pivotAngle)) {
            pivotSubsystem.setDegrees(pivotAngle);
        } else {
            pivotSubsystem.stop();
        }

        direction = getDriveAngle();

        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double calculatePivotAngle() {
        return Math.atan((AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT)
                / visionSubsystem.getDistanceTo(
                    getSpeakerPose()
                ));
    }

    private Pose2d getSpeakerPose() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ?
                        VisionConstants.BLUE_SPEAKER_CENTER :
                        VisionConstants.RED_SPEAKER_CENTER;
    }

    private int getDriveAngle() {
        // return (int) visionSubsystem.getAngleTo(getSpeakerPose()).getDegrees() ; TODO: Fix this
        return 0;
    }
}