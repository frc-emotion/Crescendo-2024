package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {
    @AutoLog
    public class SwerveIOInputs {
        public double driveSpeed = 0.0;
        public double angularSpeed = 0.0;
        public double currentAngle = 0.0;
    }
    public default void updateInputs(SwerveIOInputsAutoLogged inputs) {}
    public default void updateConstants(double Module_kP, double Module_kI, double Module_kD) {}
    public default void setModuleStates(SwerveModuleState[] states) {}
    public default void driveRobotRelative(ChassisSpeeds speeds) {}
}
