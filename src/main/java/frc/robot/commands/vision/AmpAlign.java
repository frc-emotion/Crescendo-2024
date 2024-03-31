package frc.robot.commands.vision;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AmpAlign extends MonitorVision {
    @SuppressWarnings("unused")
    private SwerveSubsystem swerveSubsystem;

    public AmpAlign(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        super(visionSubsystem);
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {

        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
