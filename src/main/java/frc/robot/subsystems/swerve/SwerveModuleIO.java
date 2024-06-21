package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        /** The current SwerveModuleState */
        public SwerveModuleState currentModuleState = new SwerveModuleState();

        /** The desired SwerveModuleState */
        public SwerveModuleState desiredModuleState = new SwerveModuleState();

        /** The current SwerveModulePosition */
        public SwerveModulePosition currentModulePosition = new SwerveModulePosition();

        /** The current position of the drive motor in rotations. */
        public double drivePosition = 0.0;

        /** The current velocity of the drive motor in rotations per minute. */
        public double driveSpeed = 0.0;

        /** The current position of the turning motor in rotations. */
        public double turnPosition = 0.0;

        /** The current speed of the turning motor in rotations per minute. */
        public double turnSpeed = 0.0;

        /** 
         * Whether or not station is enabled on the Swerve Module.
         * If it is enabled, this means that the Swerve Module will either go to the target
         * angle or go to the angle 180 degrees from the target. If the module goes to the 
         * opposite angle, then the drive motor will run in reverse to account for this. This
         * essentially just makes rotating the swerve modules to their target positions faster.
         * 
         */
        public boolean station = true;

        /** The current position of the CANCoder */
        public double absolutePosition = 0.0;
        
    }
    public default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {}
    public default void updatePID(double kP, double kI, double kD) {}
    public default void setDesiredModuleState(SwerveModuleState state, boolean station) {}
    public default void setUpdateFrequency(double frequency) {}
    public default void stop() {}
    public default void resetEncoders() {}
}
