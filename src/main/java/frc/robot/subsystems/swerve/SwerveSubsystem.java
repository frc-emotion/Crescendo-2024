package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;

// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Time;
// import edu.wpi.first.units.Voltage;
// import java.util.function.Consumer;
// import edu.wpi.first.networktables.StringPublisher;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.util.SendableNumber;
import frc.robot.util.SwerveLimiter;
import frc.robot.util.TabManager;
import frc.robot.util.SwerveLimiter.LimiterConstraints;
import frc.robot.util.TabManager.SubsystemTab;

public class SwerveSubsystem extends SubsystemBase {
    SendableNumber kPTheta = new SendableNumber(SubsystemTab.DRIVETRAIN, "Drive kP", DriveConstants.kPTheta);
    SendableNumber kITheta = new SendableNumber(SubsystemTab.DRIVETRAIN, "Drive kI", DriveConstants.kITheta);
    SendableNumber kDTheta = new SendableNumber(SubsystemTab.DRIVETRAIN, "Drive kD", DriveConstants.kDTheta);
    SendableNumber MaxSpeed = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Drive Speed", DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    SendableNumber MaxAccel = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Drive Accel", DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    SendableNumber MaxAngSpd = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Angular Speed", DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
    SendableNumber MaxAngAccel = new SendableNumber(SubsystemTab.DRIVETRAIN, "Max Angular Accel", DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    private SwerveIO io;
    private static final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    private PIDController thetaController;

    private ChassisSpeeds lastSpeeds;

    private SwerveLimiter limiter;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;
        lastSpeeds = new ChassisSpeeds();
        thetaController = new PIDController(DriveConstants.kPTheta, DriveConstants.kITheta, DriveConstants.kDTheta);

        limiter = new SwerveLimiter(DriveConstants.kNormalDriveConstraints);
    }

    public SwerveSubsystem() {
        this(new SwerveIOReal());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveSubsystem", inputs);
    }

    public void setDesiredModuleStates(SwerveModuleState[] states) {
        io.setDesiredModuleStates(states);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        lastSpeeds = limiter.calculate(speeds);
        io.driveRobotRelative(lastSpeeds);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        lastSpeeds = limiter.calculate(speeds);
        io.driveFieldRelative(lastSpeeds);
    }

    public void resetHeading() {
        io.zeroHeading();
    }

    public double getSpeed() {
        return inputs.speed;
    }

    public double getHeading() {
        return inputs.heading;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return inputs.modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        return inputs.moduleStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return lastSpeeds;
    }

    public double calculateThetaPID(int target) {
        thetaController.setSetpoint(target);
        return thetaController.calculate(getHeading());
    }

    public boolean isThetaAtSetpoint() {
        return thetaController.atSetpoint();
    } 

    public boolean isGyroCalibrating() {
        io.updateInputs(inputs);
        return inputs.isGyroCalibrating;
    }

    public void stopModules() {
        io.stopModules();
    }

    public void setMaxDriveConstraints(LimiterConstraints constraints) {
        limiter.setConstraints(constraints);
    }

    public void updatePID() {
        thetaController.setPID(
            kPTheta.get(),
            kITheta.get(),
            kDTheta.get()
        );
    }
}