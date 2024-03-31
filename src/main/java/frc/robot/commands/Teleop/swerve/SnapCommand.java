package frc.robot.commands.Teleop.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Theoretically rotates the robot to face a certain direction while not moving
 */
public class SnapCommand extends Command {

    protected int direction;
    private SwerveSubsystem swerveSubsystem;
    private ChassisSpeeds robotSpeeds;

    public SnapCommand(
            SwerveSubsystem swerveSubsystem,
            int direction) {

        this.direction = direction;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.updatePID();
    }

    @Override
    public void execute() {
        super.execute();
        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                swerveSubsystem.calculateThetaPID(swerveSubsystem.getHeading(), direction, false),
                swerveSubsystem.getRotation2d());

        swerveSubsystem.driveFieldRelative(robotSpeeds);
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.thetaPIDAtSetpoint(false);
    }
}
