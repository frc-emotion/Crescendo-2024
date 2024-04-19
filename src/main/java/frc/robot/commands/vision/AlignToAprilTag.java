package frc.robot.commands.vision;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignToAprilTag extends Command {
    private VisionSubsystem visionSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private int id;

    private double targetAngle;

    public AlignToAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, int id) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.id = id;
        targetAngle = 0;
        addRequirements(visionSubsystem, swerveSubsystem);
    }

    public AlignToAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        this(visionSubsystem, swerveSubsystem, -1);
    }

    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> target;
        if(id != -1) {
            target = visionSubsystem.getTarget(id);
            
        } else {
            target = visionSubsystem.getBestTarget();
        }

        if(target.isPresent()) {
            targetAngle = target.get().getYaw() + swerveSubsystem.getHeading();
        }

        if(targetAngle != 0) {
            swerveSubsystem.driveRobotRelative(
                new ChassisSpeeds(
                    0,
                    0,
                    swerveSubsystem.calculateThetaPID(targetAngle, id, false)
                )
            );
        }
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.thetaPIDAtSetpoint(false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
