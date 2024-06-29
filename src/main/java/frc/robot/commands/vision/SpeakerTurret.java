package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class SpeakerTurret extends Command {
    protected PivotSubsystem pivotSubsystem;
    protected VisionSubsystem visionSubsystem;

    protected boolean dynamicSpeedEnabled;

        // Distance to rpm shooter map
    private final static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

        // Static initializer
    static {
        shooterMap.clear();
        shooterMap.put(1.0, ShooterConstants.SHOOTER_SPEED_RPM);
        shooterMap.put(3.07, 4500.0);
    }

    public SpeakerTurret(VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem) {
        this(visionSubsystem, pivotSubsystem, false);
    }

    public SpeakerTurret(VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, boolean dynamicSpeedEnabled) {
        this.visionSubsystem = visionSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);

        this.dynamicSpeedEnabled = dynamicSpeedEnabled;
    }

    @Override
    public void initialize() {
        pivotSubsystem.toggleTurret();
    }

    @Override
    public void execute() {
        pivotSubsystem.setDegrees(calculateAngle());
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
     * Gives the inverse tangent of the height from the pivot to about the middle of
     * the speaker's mouth divided by the distance to the speaker.
     * Checks for team in order to determine the correct position. Uses
     * Translation2D constants for speaker position.
     * 
     * @return The target angle for the pivot.
     */
    protected double calculateAngle() {
        return Math.toDegrees(
                Math.atan(
                        (AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT) / 
                        visionSubsystem.getDistanceTo(
                                        (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                                                ? VisionConstants.RED_SPEAKER_CENTER
                                                : VisionConstants.BLUE_SPEAKER_CENTER)));
    }

    

}
