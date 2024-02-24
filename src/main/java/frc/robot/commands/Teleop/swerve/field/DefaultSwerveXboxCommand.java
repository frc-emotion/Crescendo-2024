package frc.robot.commands.Teleop.swerve.field;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultSwerveXboxCommand extends AbstractSwerveXboxCommand {
    
    public DefaultSwerveXboxCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc
        //Supplier<Boolean> slowModeFunc,
        //Supplier<Boolean> turboModeFunc,
        //Supplier<Double> hardLeft,
        //Supplier<Double> hardRight
    ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setMaxSpeeds(
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / 2,
            DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2,
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / 2
        );
    }

    @Override
    public void execute() {
        super.execute();
        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        sendSpeedsToSubsystem();
    }
}
