package frc.robot.commands.Teleop.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public abstract class AbstractSwerveXboxCommand extends Command {

    protected final SwerveSubsystem swerveSubsystem;
    protected final Supplier<Double> xSpdFunc, ySpdFunc, turningSpdFunc;

    protected final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    protected double[] speeds;
    protected double currentTranslationalSpeed, currentAngularSpeed;
    protected ChassisSpeeds robotSpeeds;
    protected double xSpeed, ySpeed, turningSpeed;

    public AbstractSwerveXboxCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc
        //Supplier<Boolean> fieldOrientedFunc
        //Supplier<Boolean> slowModeFunc,
        //Supplier<Boolean> turboModeFunc,
        //Supplier<Double> hardLeft,
        //Supplier<Double> hardRight
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.turningSpdFunc = turningSpdFunc;

        this.xLimiter =
            new SlewRateLimiter(
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
            );
        this.yLimiter =
            new SlewRateLimiter(
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
            );
        this.turningLimiter =
            new SlewRateLimiter(
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
            );

        robotSpeeds = new ChassisSpeeds();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        xSpeed = xSpdFunc.get();
        ySpeed = ySpdFunc.get();
        turningSpeed = turningSpdFunc.get();
        
        speeds = swerveSubsystem.getSpeedType();

        currentTranslationalSpeed = speeds[0];
        currentAngularSpeed = speeds[1];

        // deadBand
        xSpeed = Math.abs(xSpeed) > (OIConstants.kDeadband / 2) ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > (OIConstants.kDeadband / 2) ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * currentTranslationalSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * currentTranslationalSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * currentAngularSpeed;

        sendSpeedsToSubsystem();
    }

    protected void sendSpeedsToSubsystem() {
        robotSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        swerveSubsystem.setChassisSpeeds(robotSpeeds);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
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
