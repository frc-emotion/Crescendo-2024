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
        visionSubsystem.setVisionType(visionSubsystem.getSelectedType());
    }

    @Override
    public void execute() {
        visionSubsystem.updateEstimator();

        switch (visionSubsystem.getSelectedType()) {
            case NO_VISION:
                break;
            case TORPEDO:
                visionSubsystem.updateVision3();
                break;
            case PRESET_STD:
                visionSubsystem.updateVision1();
                break;
            case CUSTOM_STD:
                visionSubsystem.updateVision2();
                break;
            case CRAZY_MATH:
                visionSubsystem.updateVision4();
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
