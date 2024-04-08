package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SlowModeSwerveCommand extends DefaultSwerveXboxCommand {

    public SlowModeSwerveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc,
            Supplier<Double> ySpdFunc,
            Supplier<Double> turningSpdFunc,
            Supplier<Boolean> isFieldCentricFunc
    // Supplier<Boolean> turboModeFunc,
    // Supplier<Double> hardLeft,
    // Supplier<Double> hardRight
    ) {
        super(
                swerveSubsystem,
                xSpdFunc,
                ySpdFunc,
                turningSpdFunc,
                isFieldCentricFunc);
    }

    /**
     * Limits the maximum speed and acceleration to a quarter of the maximum
     */
    @Override
    public void initialize() {
        DriveConstants.currentDriveMode = DriveMode.SLOW;
        swerveSubsystem.setMaxSpeeds(
                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 4,
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / 4,
                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4,
                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / 4);
    }
}