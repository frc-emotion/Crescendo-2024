package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PivotAutoAimCommand extends Command {
    private PivotSubsystem pivotSubsystem;
    private VisionSubsystem visionSubsystem;

    public PivotAutoAimCommand(PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        pivotSubsystem.setRev(calculatePivotAngle());
    }

    private double calculatePivotAngle() {
        double distance = visionSubsystem.getDistanceTo(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? VisionConstants.BLUE_SPEAKER_CENTER : VisionConstants.RED_SPEAKER_CENTER);
        return Math.atan(
            (AutoConstants.SPEAKER_MOUTH_HEIGHT - AutoConstants.PIVOT_HEIGHT) / 
            distance
        );
    }
}
