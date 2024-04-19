package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DefaultSwerveXboxCommand extends AbstractSwerveXboxCommand {
    protected final Supplier<Boolean> isRobotCentricFunc;

    public DefaultSwerveXboxCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc,
            Supplier<Boolean> isRobotCentricFunc
    // Supplier<Boolean> slowModeFunc,
    // Supplier<Boolean> turboModeFunc,
    // Supplier<Double> hardLeft,
    // Supplier<Double> hardRight
    ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc);
        this.isRobotCentricFunc = isRobotCentricFunc;
    }

    /**
     * Limits the maximum speed and acceleration of the robot to half of the maximum
     */
    @Override
    public void initialize() {
        constraints = DriveConstants.kNormalDriveConstraints;
        DriveConstants.currentDriveMode = DriveMode.NORMAL;
        swerveSubsystem.setMaxDriveConstraints(constraints);
    }

    @Override
    public void execute() {
        super.execute();
        if (!isRobotCentricFunc.get()) {
            DriveConstants.isRobotCentric = false;
        } else {
            DriveConstants.isRobotCentric = true;
            robotSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        sendSpeedsToSubsystem();
    }

    /**
     * Restores the maximum speed of the robot to the normal mode speeds. Mainly
     * used in subclasses.
     * Stops the drivetrain if interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveSubsystem.stopModules();
        }
        swerveSubsystem.setMaxDriveConstraints(DriveConstants.kNormalDriveConstraints);
    }
}
