package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double turnPos = 0;
        public double drivePos = 0.0;
        public double turnSpeed = 0.0;
        public double driveSpeed = 0.0; 
        public double absolutePos = 0.0;
    }

    public void updateInputs(SwerveModuleIOInputs inputs);
    public void updateConstants(double kPTurning, double kITurning, double kDTurning, double kPDrive, double kIDrive, double kDDrive, double kSDrive, double kVDrive);
    public void setDesiredModuleState(SwerveModuleState state);
    public void setReferenceAngle(double angle);
    public void stop();
}
