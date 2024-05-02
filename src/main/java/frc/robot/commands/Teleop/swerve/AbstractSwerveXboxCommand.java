package frc.robot.commands.Teleop.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.SwerveLimiter.LimiterConstraints;

import java.util.function.Supplier;

public abstract class AbstractSwerveXboxCommand extends Command {
    protected final SwerveSubsystem swerveSubsystem;
    protected final Supplier<Double> xSpdFunc, ySpdFunc, turningSpdFunc;

    protected ChassisSpeeds robotSpeeds;
    protected double xSpeed, ySpeed, turningSpeed;
    protected LimiterConstraints constraints;

    @Override
    public void initialize() {
        constraints = DriveConstants.kNormalDriveConstraints;
        swerveSubsystem.setMaxDriveConstraints(constraints);
    }

    public AbstractSwerveXboxCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc
    // Supplier<Boolean> fieldOrientedFunc
    // Supplier<Boolean> slowModeFunc,
    // Supplier<Boolean> turboModeFunc,
    // Supplier<Double> hardLeft,
    // Supplier<Double> hardRight
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.turningSpdFunc = turningSpdFunc;

        robotSpeeds = new ChassisSpeeds();

        addRequirements(swerveSubsystem);
    }

    /**
     * Processes user input and limits the acceleration of the robot using slew
     * limiters.
     */
    @Override
    public void execute() {
        xSpeed = xSpdFunc.get();
        ySpeed = ySpdFunc.get();
        turningSpeed = turningSpdFunc.get();

        // Retrieves the user input and applies a deadzone
        xSpeed = Math.abs(xSpeed) > (OIConstants.kDeadband / 2) ? constraints.maxSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > (OIConstants.kDeadband / 2) ? constraints.maxSpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? constraints.maxAngularSpeed : 0.0;

        robotSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds, swerveSubsystem.getRotation2d());
    }

    /**
     * Sends the robot-relative ChassisSpeeds to the Swerve Subsystem.
     */
    protected void sendSpeedsToSubsystem() {

        // swerveSubsystem.setChassisSpeeds(robotSpeeds);
        // SwerveModuleState[] moduleStates =
        // DriveConstants.kDriveKinematics.toSwerveModuleStates(robotSpeeds);
        // swerveSubsystem.setModuleStates(moduleStates);

        swerveSubsystem.driveRobotRelative(robotSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
