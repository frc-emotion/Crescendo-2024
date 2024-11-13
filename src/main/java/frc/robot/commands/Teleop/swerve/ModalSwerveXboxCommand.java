package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.SwerveSubsystem;

public class ModalSwerveXboxCommand extends Command {
    protected final SwerveSubsystem swerveSubsystem;
    protected final Supplier<Double> xSpdFunc, ySpdFunc, turningSpdFunc;

    protected SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    protected final DriveMode driveMode;
    
    protected double maxDriveSpeed;
    protected double maxAngularSpeed;

    public ModalSwerveXboxCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc,
        DriveMode mode
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.turningSpdFunc = turningSpdFunc;
        this.driveMode = mode;

        switch(driveMode) {
            case TURBO:
                xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                maxDriveSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                maxAngularSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
                break;
                case SLOW:
                xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveSlowAccelerationUnitsPerSecond);
                yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveSlowAccelerationUnitsPerSecond);
                turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveSlowAngularAccelerationUnitsPerSecond);
                maxDriveSpeed = DriveConstants.kTeleDriveSlowSpeedMetersPerSecond;
                maxAngularSpeed = DriveConstants.kTeleDriveSlowAngularSpeedRadiansPerSecond;
                break;
            default:
                xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveNormalAccelerationUnitsPerSecond);
                yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveNormalAccelerationUnitsPerSecond);
                turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveSlowAngularAccelerationUnitsPerSecond);
                maxDriveSpeed = DriveConstants.kTeleDriveNormalSpeedMetersPerSecond;
                maxAngularSpeed = DriveConstants.kTeleDriveNormalAngularSpeedRadiansPerSecond;
        }
    }

    @Override
    public void initialize() {
        DriveConstants.currentDriveMode = this.driveMode;
    }

    @Override
    public void execute() {
        double x = xLimiter.calculate(xSpdFunc.get() * maxDriveSpeed);
        double y = yLimiter.calculate(ySpdFunc.get() * maxDriveSpeed);
        double theta = turningLimiter.calculate(turningSpdFunc.get() * maxAngularSpeed);
        
        swerveSubsystem.driveRobotRelative(prepareSpeeds(new ChassisSpeeds(x, y, theta)));
    }

    protected ChassisSpeeds prepareSpeeds(ChassisSpeeds speeds) {
        return speeds;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
