package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SwerveLimiter;
import frc.robot.util.SwerveLimiter.LimiterConstraints;

public class SwerveIOReal implements SwerveIO {
    private SwerveModuleNeo[] modules;

    private AHRS gyro;

    private ChassisSpeeds lastSpeeds;

    public SwerveIOReal() {
        modules = new SwerveModuleNeo[4];
        for(int i = 0; i < 4; i++) {
            modules[i] = new SwerveModuleNeo(i);
        }
        gyro = new AHRS(SPI.Port.kMXP);
        lastSpeeds = new ChassisSpeeds();
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.heading = getHeading();
        inputs.turnSpeed = lastSpeeds.omegaRadiansPerSecond;
        inputs.speed = getSpeed();
        inputs.isGyroCalibrating = gyro.isCalibrating();
        inputs.modulePositions = getModulePositions();
        inputs.moduleStates = getModuleStates();
        for(int i = 0; i < 4; i++) {
            modules[i].update();
        }
    }

    @Override
    public void setDesiredModuleStates(SwerveModuleState[] states) {
        for(int i = 0; i < 4; i++) {
            modules[i].setDesiredModuleState(states[i]);
        }
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.lastSpeeds = speeds;
        var moduleStates = toModuleStates(speeds);
        setDesiredModuleStates(moduleStates);
    }

    @Override
    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2d()));
    }

    private SwerveModuleState[] toModuleStates(ChassisSpeeds speeds) {
        return DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    private ChassisSpeeds toChassisSpeeds(SwerveModuleState[] states) {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }

    private Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    private double getHeading() {
        return gyro.getAngle();
    }

    private double getSpeed() {
        return Math.sqrt(Math.pow(lastSpeeds.vxMetersPerSecond, 2) + Math.pow(lastSpeeds.vyMetersPerSecond, 2));
    }

    @Override
    public void zeroHeading() {
        gyro.reset();
    }

    @Override
    public void stopModules() {
        for(int i = 0; i < 4; i++) {
            modules[i].stop();
        }
    }
    
}

