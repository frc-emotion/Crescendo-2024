package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

/**
 * Note: Uses analog absolute encoder instead of a CANCoder
 *
 */
public class SwerveModuleNeo {
    private SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private int index;

    public SwerveModuleNeo(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    /**
     * Index to wheel chart
     * 1 - Front Left
     * 2- Front Right
     * 3 - Back Left
     * 4 - Back Right
     * 
     * @param index The index of the new swerve module object
     */
    public SwerveModuleNeo(int index) {
        switch(index) {
            case 0:
                io = new SwerveModuleIONeo(
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
                );
                break;
            case 1:
                io = new SwerveModuleIONeo(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
                );
                break;
            case 2:
                io = new SwerveModuleIONeo(
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackLeftTurningEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
                );
                break;

            case 3:
                io = new SwerveModuleIONeo(
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackRightTurningEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed
                );
                break;
            default:
                throw new UnsupportedOperationException("No Swerve Module ID set");
            
        }

        this.index = index;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveModule" + index, inputs);
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

    public SwerveModuleState getCurrentState() {
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

    public void setDesiredModuleState(SwerveModuleState state) {
        io.setDesiredModuleState(state, false);
    }

    public void setDesiredModuleState(SwerveModuleState state, boolean station) {
        io.setDesiredModuleState(state, station);
    }

    public void resetEncoders() {
        io.resetEncoders();
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
