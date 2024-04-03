package frc.robot.commands.Auto.SubsystemCommands;

import frc.robot.commands.vision.SpeakerTurret;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoSpeakerTurret extends SpeakerTurret {
    public AutoSpeakerTurret(VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem) {
        super(visionSubsystem, pivotSubsystem);
        
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.isAtTarget(calculateAngle());
    }
}
