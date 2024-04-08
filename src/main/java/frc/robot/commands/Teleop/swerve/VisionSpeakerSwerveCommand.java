package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.other.VisionSubsystem;
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
        direction = (int) (swerveSubsystem.getHeading() + visionSubsystem.getTX());

        double angle = calculateAngle();
        if (!pivotSubsystem.isAtTarget(angle)) {
            pivotSubsystem.setRev(angle);
        } else {
            pivotSubsystem.stop();
        }

        super.execute();
    }

    private double calculateAngle() {
        return Math.atan((AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT)
                / visionSubsystem.getDistanceTo(VisionConstants.BLUE_SPEAKER_CENTER));
    }
}