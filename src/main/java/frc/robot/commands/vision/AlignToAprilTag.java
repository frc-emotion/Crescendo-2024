package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTag extends Command {
    private VisionSubsystem visionSubsystem;
    private SwerveSubsystem swerveSubsystem;

    public AlignToAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
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
