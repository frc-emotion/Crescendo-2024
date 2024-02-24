package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SlowModeSwerveCommand extends DefaultSwerveXboxCommand {

    public SlowModeSwerveCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc
        //Supplier<Boolean> turboModeFunc,
        //Supplier<Double> hardLeft,
        //Supplier<Double> hardRight
    ) {
        super(
            swerveSubsystem,
            xSpdFunc,
            ySpdFunc,
            turningSpdFunc
        );
    }

    @Override
    public void initialize() {
        swerveSubsystem.setMaxSpeeds(
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 4,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / 4,
            DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4,
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / 4
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setMaxSpeeds(
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / 2,
            DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2,
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / 2
        );
    }
    
}