package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SpeakerTurret extends Command {
    private VisionSubsystem visionSubsystem;
    private PivotSubsystem pivotSubsystem;

    public SpeakerTurret(VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.toggleTurret();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !pivotSubsystem.turretMode();
    }

    @Override
    public void end(boolean interrupted) {

    }

}
