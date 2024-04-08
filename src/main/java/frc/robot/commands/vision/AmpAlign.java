package frc.robot.commands.vision;

import frc.robot.subsystems.other.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
