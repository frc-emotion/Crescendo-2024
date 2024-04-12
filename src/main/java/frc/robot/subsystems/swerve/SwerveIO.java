package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.SwerveLimiter.LimiterConstraints;

public interface SwerveIO {
    @AutoLog
    public class SwerveIOInputs {
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
        public ChassisSpeeds robotSpeeds = new ChassisSpeeds();
        public double driveSpeed = 0.0;
        public double angularSpeed = 0.0;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double roll = 0.0;
        public boolean isGyroCalibrating = false;
    }
    public default void updateInputs(SwerveIOInputs inputs) {}
    public default void updateConstants(double Module_kP, double Module_kI, double Module_kD) {}
    public default void updateDriveConstraints(LimiterConstraints constraints) {}
    public default void setModuleStates(SwerveModuleState[] states) {}
    public default void driveRobotRelative(ChassisSpeeds speeds) {}
    public default void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {}
    public default void setGyroOffset(double offset) {}
    public default void resetGyro() {}
    public default void stop() {}
}
