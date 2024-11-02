package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Theoretically rotates the robot to face a certain direction.
 * TODO: Fix overshooting
 */
public class SnapSwerveCommand extends AbstractSwerveXboxCommand {

    protected int direction;

    public SnapSwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc,
            int direction) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc);

        this.direction = direction;
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get() != DriverStation.Alliance.Blue) {
            this.direction = -this.direction;
        }
        swerveSubsystem.updatePID();
    }

    @Override
    public void execute() {
        super.execute();
        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                swerveSubsystem.calculateThetaPID(
                    swerveSubsystem.getHeading(), 
                    direction, 
                    false
                ),
                swerveSubsystem.getRotation2d());

        sendSpeedsToSubsystem();
        // System.out.println(robotSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
