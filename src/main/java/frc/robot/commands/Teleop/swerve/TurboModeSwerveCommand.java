package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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

    /**
     * Sets the maximum speed and acceleration of the robot to the maximum possible
     * speed and acceleration.
     */
    @Override
    public void initialize() {
        DriveConstants.currentDriveMode = DriveMode.TURBO;
        swerveSubsystem.setMaxSpeeds(
                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }
}
