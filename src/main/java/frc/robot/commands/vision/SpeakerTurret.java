package frc.robot.commands.vision;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SpeakerTurret extends MonitorVision {
    private PivotSubsystem pivotSubsystem;

    private final static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    static {
        shooterMap.clear();
        shooterMap.put(1.46, 61.9);
     
    }

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
        //System.out.println(calculateAngle());
        //System.out.println("blud");
        System.out.println(visionSubsystem.getDistanceTo((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? VisionConstants.RED_SPEAKER_CENTER : VisionConstants.BLUE_SPEAKER_CENTER));
        pivotSubsystem.setRev(calculateAngle());
    }

    @Override
    public boolean isFinished() {
        // return pivotSubsystem.isAtTarget(calculateAngle());
        return false;
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
        return Math.toDegrees(
            Math.atan(
                (AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT) 
                / visionSubsystem.getDistanceTo(
            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? VisionConstants.RED_SPEAKER_CENTER : VisionConstants.BLUE_SPEAKER_CENTER
        )));
    }

}
