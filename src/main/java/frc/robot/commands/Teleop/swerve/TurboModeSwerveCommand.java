package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.SwerveSubsystem;

public class TurboModeSwerveCommand extends DefaultSwerveXboxCommand {

    public TurboModeSwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc,
            Supplier<Boolean> isFieldCentricFunc
    // Supplier<Boolean> slowModeFunc,
    // Supplier<Boolean> turboModeFunc,
    // Supplier<Double> hardLeft,
    // Supplier<Double> hardRight
    ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc, isFieldCentricFunc);
    }

    @Override
    public void initialize() {
        DriveConstants.currentDriveMode = DriveMode.TURBO;
        swerveSubsystem.setMaxSpeeds(
                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveSubsystem.stopModules();
        }

        swerveSubsystem.setMaxSpeeds(
                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2,
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / 2,
                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2,
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / 2);
    }
}
