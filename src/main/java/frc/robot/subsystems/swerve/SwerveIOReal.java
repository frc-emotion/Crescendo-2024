package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveIOReal implements SwerveIO {
    private SwerveModuleNeo[] swerveModules;
    
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private ChassisSpeeds robotSpeeds;
    private double targetAngle;

    public SwerveIOReal(SwerveModuleNeo frontLeft, SwerveModuleNeo frontRight, SwerveModuleNeo backLeft, SwerveModuleNeo backRight) {
        swerveModules[0] = frontLeft;
        swerveModules[1] = frontRight;
        swerveModules[2] = backLeft;
        swerveModules[3] = backRight;

        robotSpeeds = new ChassisSpeeds();
        targetAngle = 0;
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        for(int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredModuleState(states[i], true);
        }
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
        robotSpeeds = speeds;
        setModuleStates(getModuleStates(speeds));
    }

    @Override
    public void updateInputs(SwerveIOInputsAutoLogged inputs) {
        inputs.driveSpeed = getSpeed();
        inputs.angularSpeed = getAngularSpeed();
        inputs.currentAngle = getHeading();
        
        for(SwerveModuleNeo module : swerveModules) {
            module.updateInputs();
        }
    }

    @Override
    public void updateConstants(double Module_kP, double Module_kI, double Module_kD) {
        for(SwerveModuleNeo module : swerveModules) {
            module.updatePID(Module_kP, Module_kI, Module_kD);
        }
    }

    public double getAngularSpeed() {
        return gyro.getRate();
    }

    public double getHeading() {
        return gyro.getAngle();
    }

    public double getSpeed() {
        return Math.sqrt(Math.pow(robotSpeeds.vxMetersPerSecond, 2) + Math.pow(robotSpeeds.vyMetersPerSecond, 2));
    }

    public SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
        return DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    
}
