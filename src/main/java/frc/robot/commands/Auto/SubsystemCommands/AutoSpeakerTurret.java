package frc.robot.commands.Auto.SubsystemCommands;

import frc.robot.commands.vision.SpeakerTurret;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class AutoSpeakerTurret extends SpeakerTurret {
    public AutoSpeakerTurret(VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem) {
        super(visionSubsystem, pivotSubsystem);
        
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.isAtTarget(calculateAngle());
    }
}
