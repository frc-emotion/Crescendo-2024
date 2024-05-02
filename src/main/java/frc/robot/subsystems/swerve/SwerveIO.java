package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.SwerveLimiter.LimiterConstraints;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        public double speed = 0.0;
        public double turnSpeed = 0.0;
        public double heading = 0.0;
        public boolean isGyroCalibrating = false;

        public SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        public SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    }

    public void updateInputs(SwerveIOInputs inputs);
    public void setDesiredModuleStates(SwerveModuleState[] states);
    public void driveRobotRelative(ChassisSpeeds speeds);
    public void driveFieldRelative(ChassisSpeeds speeds);
    public void zeroHeading();
    public void stopModules();
}