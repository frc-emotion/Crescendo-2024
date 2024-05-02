package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.VisionSubsystem;

public class MonitorVision extends Command {

    protected final VisionSubsystem visionSubsystem;

    public MonitorVision(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.setVisionType(visionSubsystem.getSelectedType());
    }

    @Override
    public void execute() {
        visionSubsystem.updateOdometry();

        switch (visionSubsystem.getSelectedType()) {
            case NO_VISION:
                break;
            case CUSTOM_STD:
                visionSubsystem.updateVision2();
                break;
            case LLDOCS:
                visionSubsystem.updateVision5();
                break;
            default:
                break;
        }

        visionSubsystem.updateNetworkTable();
        visionSubsystem.updateField();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
