package frc.robot.commands.vision;

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
        super.execute();
        pivotSubsystem.setRev(calculateAngle());
    }

    @Override
    public boolean isFinished() {
        return !pivotSubsystem.turretMode();
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stop();
    }

    public double calculateAngle() {
        return Math.atan((AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT) / visionSubsystem.getDistanceTo(VisionConstants.BLUE_SPEAKER_CENTER));
    }

}
