package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {

    private final VisionSubsystem visionSubsystem;

    public VisionCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.setVisionType(visionSubsystem.getSelectedType());
    }

    @Override
    public void execute() {
        visionSubsystem.updateEstimator();

        switch (visionSubsystem.getSelectedType()) {
            case NO_VISION:
                break;
            case CUSTOM_STD:
                visionSubsystem.updateVision2();
                break;
            case LLDOCS:
                visionSubsystem.updateVision5();
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