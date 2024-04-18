package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.SwerveLimiter.LimiterConstraints;

public interface SwerveIO {
    @AutoLog
    public class SwerveIOInputs {
        /** The current SwerveModuleStates for all modules. */
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
        
        /** The current SwerveModulePositions for all modules. */
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
        
        /** The current ChassisSpeeds being sent the subsystem. */
        public ChassisSpeeds robotSpeeds = new ChassisSpeeds();

        /** The current drive speed in meters per second. */
        public double driveSpeed = 0.0;

        /** The current rotational speed in degrees per second? */
        public double angularSpeed = 0.0;

        /** The current heading in degrees. */
        public double yaw = 0.0;

        /** The current pitch in degrees. */
        public double pitch = 0.0;

        /** The current roll in degrees. */
        public double roll = 0.0;

        /** Whether or not the gyro is currently calibrating. */
        public boolean isGyroCalibrating = false;
    }
    /** Updates the SwerveIOInputs with the current variable states */
    public default void updateInputs(SwerveIOInputs inputs) {}

    /**
     * Updates the Swerve Module PID Constants
     * @see frc.robot.subsystems.swerve.SwerveModuleIO#updatePID(double, double, double)
     */
    public default void updateConstants(double Module_kP, double Module_kI, double Module_kD) {}

    /**
     * Updates the current Drive Constraints as set by the {@link frc.robot.util.SwerveLimiter}
     * 
     * @param constraints The {@link frc.robot.util.SwerveLimiter.LimiterConstraints} object which stores the next constraints to use.
     */
    public default void updateDriveConstraints(LimiterConstraints constraints) {}

    /**
     * Sets the Swerve Module States
     * @param states The target states to set
     * @see SwerveModuleState
     */
    public default void setModuleStates(SwerveModuleState[] states) {}

    /**
     * Sets the robot to drive relative to itself.
     * @param speeds The {@link ChassisSpeeds} object to use
     */
    public default void driveRobotRelative(ChassisSpeeds speeds) {}

    /**
     * Sets the robot to drive relative to the field
     * @param fieldRelativeSpeeds The {@link ChassisSpeeds} object to use
     */
    public default void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {}

    /**
     * Sets the offset of the gyro.
     * @param offset The amount to offset the gyro.
     */
    public default void setGyroOffset(double offset) {}

    /**
     * Resets the Gyro heading to zero.
     */
    public default void resetGyro() {}

    /**
     * Fully stops the entire Swerve Subsystem
     */
    public default void stop() {}
}
