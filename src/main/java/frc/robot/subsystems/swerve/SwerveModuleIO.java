package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public SwerveModuleState currentModuleState = new SwerveModuleState();
        public SwerveModuleState desiredModuleState = new SwerveModuleState();

        public SwerveModulePosition currentModulePosition = new SwerveModulePosition();

        public double drivePosition = 0.0;
        public double driveSpeed = 0.0;

        public double turnPosition = 0.0;
        public double turnSpeed = 0.0;
        public boolean station = false;

        public double absolutePosition = 0.0;
        
    }
    public default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {}
    public default void updatePID(double kP, double kI, double kD) {}
    public default void setDesiredModuleState(SwerveModuleState state, boolean station) {}
    public default void setUpdateFrequency(double frequency) {}
    public default void stop() {}
    public default void resetEncoders() {}
}
