package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Note: Uses analog absolute encoder instead of a CANCoder
 *
 */
public class SwerveModuleNeo {
    private SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModuleNeo(SwerveModuleIO io) {
        this.io = io;
    }

    public SwerveModuleIOInputsAutoLogged getModuleInputs() {
        return inputs;
    }

    public void setDesiredState(SwerveModuleState state, boolean station) {
        io.setDesiredModuleState(state, station);
    }

    public void updatePID(double kP, double kI, double kD) {
        io.updatePID(kP, kI, kD);
    }

    public SwerveModulePosition getPosition() {
        return inputs.currentModulePosition;
    }

    public SwerveModuleState getState() {
        return inputs.currentModuleState;
    }

    public SwerveModuleState getDesiredState() {
        return inputs.desiredModuleState;
    }

    public double getAbsolutePosition() {
        return inputs.absolutePosition;
    }

    public double getTurningPosition() {
        return inputs.turnPosition;
    }

    public double getDriveVelocity() {
        return inputs.driveSpeed;
    }

    public void stop() {
        io.stop();
    }

    // Unit Conversion Methods
    // private double toRot(double ticks) {
    // return ticks / 2048;
    // }

    // private double toRPM(double ticks_per_time) {
    // return (ticks_per_time / 2048) * 600;
    // }

    // private double toMeter(double rot) {
    // return rot * (ModuleConstants.kDriveEncoderRot2Meter);
    // }

    // private double toMPS(double rpm) {
    // return rpm * (ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    // }

    // public static double RPMToFalcon(double RPM, double gearRatio) {
    // double motorRPM = RPM * gearRatio;
    // double sensorCounts = motorRPM * (2048.0 / 600.0);
    // return sensorCounts;
    // }

    // public static double MPSToFalcon(double velocity, double circumference,
    // double gearRatio) {
    // double wheelRPM = ((velocity * 60) / circumference);
    // double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    // return wheelVelocity;
    // }
}
