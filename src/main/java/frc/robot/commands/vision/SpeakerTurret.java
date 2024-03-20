package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SpeakerTurret extends MonitorVision {
    private PivotSubsystem pivotSubsystem;

    public SpeakerTurret(VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem) {
        super(visionSubsystem);
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        pivotSubsystem.toggleTurret();
    }

    @Override
    public void execute() {
        //super.execute();
        pivotSubsystem.setRev(calculateAngle());
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.isAtTarget(calculateAngle());
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stop();
    }

    /**
     * Gives the inverse tangent of the height from the pivot to about the middle of the speaker's mouth divided by the distance to the speaker.
     * Checks for team in order to determine the correct position. Uses Translation2D constants for speaker position.
     * 
     * @return The target angle for the pivot.
     */
    private double calculateAngle() {
        return 60.0 - Math.atan((AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT) / visionSubsystem.getOdoDistanceTo(
            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? VisionConstants.RED_SPEAKER_CENTER : VisionConstants.BLUE_SPEAKER_CENTER
        ));
    }

}
