package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class MonitorVision extends Command {

    protected final VisionSubsystem visionSubsystem;

    public MonitorVision(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
