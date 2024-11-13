package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Theoretically rotates the robot to face a certain direction.
 * TODO: Fix overshooting
 */
public class SnapSwerveCommand extends ModalSwerveXboxCommand {

    protected int direction;

    public SnapSwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc,
            int direction) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc, DriveMode.NORMAL);

        this.direction = direction;
    }

    @Override
    protected ChassisSpeeds prepareSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds newSpeeds = new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            swerveSubsystem.calculateThetaPID(Rotation2d.fromDegrees(direction))
        );
        return newSpeeds;
    }
}
