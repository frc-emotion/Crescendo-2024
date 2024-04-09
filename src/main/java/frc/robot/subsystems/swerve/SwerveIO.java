package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {
    @AutoLog
    public class SwerveIOInputs {
        public double driveSpeed = 0.0;
        public double angularSpeed = 0.0;
        public double targetAngle = 0.0;
        public boolean atTargetAngle = true;
    }
    public default void updateInputs(SwerveIOInputsAutoLogged inputs) {}
    public default void updateConstants(double kP, double kI, double kD, double Module_kP, double Module_kI, double Module_kD) {}
    public default void setModuleStates(SwerveModuleState[] states) {}
}
