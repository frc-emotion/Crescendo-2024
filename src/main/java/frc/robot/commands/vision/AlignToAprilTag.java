package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTag extends MonitorVision {
    private VisionSubsystem visionSubsystem;
    private SwerveSubsystem swerveSubsystem;

    public AlignToAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
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
        return isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    private boolean isAligned() {
        return false;
    }
}
