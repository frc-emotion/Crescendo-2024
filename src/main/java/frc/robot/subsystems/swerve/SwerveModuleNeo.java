package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleNeo {
    private SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModuleNeo(int id) {
        io = new SwerveModuleIONeo(id);
    }

    public void update() {
        io.updateInputs(inputs);
    }

    public SwerveModuleNeo(SwerveModuleIO io) {
        this.io = io;
    }

    public void setDesiredModuleState(SwerveModuleState state) {
        io.setDesiredModuleState(state);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getTurnPosition()));
    }
    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurnPosition()));
    }

    public double getDrivePosition() {
        return inputs.drivePos;
    }

    public double getDriveSpeed() {
        return inputs.drivePos;
    }

    public double getTurnPosition() {
        return inputs.turnPos;
    }

    public void stop() {
        io.stop();
    }
}
