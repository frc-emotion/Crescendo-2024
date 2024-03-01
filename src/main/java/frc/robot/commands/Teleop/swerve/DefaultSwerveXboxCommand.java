package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultSwerveXboxCommand extends AbstractSwerveXboxCommand {
    protected final Supplier<Boolean> isRobotCentricFunc;
    
    public DefaultSwerveXboxCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc,
        Supplier<Boolean> isRobotCentricFunc
        //Supplier<Boolean> slowModeFunc,
        //Supplier<Boolean> turboModeFunc,
        //Supplier<Double> hardLeft,
        //Supplier<Double> hardRight
    ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc);
        this.isRobotCentricFunc = isRobotCentricFunc;
    }

    @Override
    public void initialize() {
        DriveConstants.currentDriveMode = DriveMode.NORMAL;
        swerveSubsystem.setMaxSpeeds(
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond / 2,
            DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2,
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond / 2
        );
    }

    @Override
    public void execute() {
        if(!isRobotCentricFunc.get()) {
            DriveConstants.isRobotCentric = false;
            super.execute();
            sendSpeedsToSubsystem();
        } else {
            DriveConstants.isRobotCentric = true;
            super.execute();
            robotSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            sendSpeedsToSubsystem();
        }
    }
}
