package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SwerveLimiter;
import frc.robot.util.SwerveLimiter.LimiterConstraints;

public class SwerveIOReal implements SwerveIO {
    private SwerveModuleNeo[] swerveModules;
    
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private ChassisSpeeds robotSpeeds;
    private double targetAngle;

    private SwerveLimiter limiter;

    /** Initializes the swerve modules using the provided {@link SwerveModuleNeo} objects. */
    public SwerveIOReal(SwerveModuleNeo frontLeft, SwerveModuleNeo frontRight, SwerveModuleNeo backLeft, SwerveModuleNeo backRight) {
        this.swerveModules = new SwerveModuleNeo[] {frontLeft, frontRight, backLeft, backRight};
        // swerveModules[0] = frontLeft;
        // swerveModules[1] = frontRight;
        // swerveModules[2] = backLeft;
        // swerveModules[3] = backRight;

        this.robotSpeeds = new ChassisSpeeds();
        this.targetAngle = 0;

        this.limiter = new SwerveLimiter(DriveConstants.kNormalDriveConstraints);
    }

    /**
     * Default constructor, initializes all the {@link SwerveModuleNeo} objects in their respective locations.
     * @see SwerveModuleNeo#SwerveModuleNeo(int)
     */
    public SwerveIOReal() {
        this(
            new SwerveModuleNeo(0),
            new SwerveModuleNeo(1),
            new SwerveModuleNeo(2),
            new SwerveModuleNeo(3)
        );
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        for(int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredModuleState(states[i], true);
        }
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // robotSpeeds = limiter.calculate(speeds);
        setModuleStates(convertToModuleStates(speeds));
    }

    @Override
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveFieldRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, Rotation2d.fromDegrees(getHeading())));
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.robotSpeeds = robotSpeeds;
        inputs.modulePositions = getModulePositions();
        inputs.moduleStates = getModuleStates();
        inputs.driveSpeed = getSpeed();
        inputs.angularSpeed = getAngularSpeed();
        inputs.yaw = getHeading();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.isGyroCalibrating = gyro.isCalibrating();

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

    @Override
    public void updateDriveConstraints(LimiterConstraints constraints) {
        limiter.setConstraints(constraints);
    }

    @Override
    public void setGyroOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void stop() {
        for(int i = 0; i < 4; i++) {
            swerveModules[i].stop();
        }
    }

    /** Retrieves the current angular speed from the gyro in degrees per second. Only used when updating the {@link SwerveIOInputs}. */
    public double getAngularSpeed() {
        return gyro.getRate();
    }

    /** Retrieves the current heading of the robot in degrees. Only used when updating the {@link SwerveIOInputs}. */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getSpeed() {
        return Math.sqrt(Math.pow(robotSpeeds.vxMetersPerSecond, 2) + Math.pow(robotSpeeds.vyMetersPerSecond, 2));
    }

    /** Converts between {@link ChassisSpeeds} and {@link SwerveModuleState} objects. */
    public SwerveModuleState[] convertToModuleStates(ChassisSpeeds speeds) {
        return DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    /** Converts between {@link SwerveModuleState} objects and {@link ChassisSpeeds}. */
    public ChassisSpeeds convertToChassisSpeeds(SwerveModuleState[] states) {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }

    /** Retrieves the current {@link SwerveModuleState} objects from the modules. Only used when updating the {@link SwerveIOInputs}. */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getCurrentState();
        }
        return states;
    }

    /** Retrieves the current {@link SwerveModulePosition} objects from the modules. Only used when updating the {@link SwerveIOInputs}. */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /** Retrieves the current ChassisSpeeds. Is automatically logged by AdvantageKit as an output. */
    @AutoLogOutput(key = "Drive/ChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return convertToChassisSpeeds(getModuleStates());
    }

    /** Fills the Shuffleboard Layouts for an individual swerve module */
    public void fillList(int index, ShuffleboardLayout layout) {
        SwerveModuleNeo module = swerveModules[index];
        layout.addNumber(
                "Absolute Position",
                () -> module.getAbsolutePosition());
        layout.addNumber(
                "Integrated Position",
                () -> module.getTurningPosition());
        layout.addNumber("Velocity", () -> module.getDriveVelocity());
        layout.withSize(2, 4);
    }
}
