package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor, feederMotor;
    private final SparkPIDController controller;

    private final RelativeEncoder shooterEncoder, feederEncoder;

    private GenericEntry ampShooterRPM;

    private DigitalInput breakSensor;

    private double targetRPM;

    public ShooterSubsystem() {
        shooterMotor =
            new CANSparkMax(ShooterConstants.shooterPort, MotorType.kBrushless);

        shooterMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_SMART);
        shooterMotor.setSecondaryCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        shooterMotor.setIdleMode(IdleMode.kCoast);

        feederMotor =
            new CANSparkMax(ShooterConstants.feederPort, MotorType.kBrushless);

        feederMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_SMART);
        feederMotor.setSecondaryCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setInverted(true);

        shooterEncoder = shooterMotor.getEncoder();
        //shooterEncoder.setVelocityConversionFactor(2);
        
        feederEncoder = feederMotor.getEncoder();

        // Set up PID Controller constants
        controller = shooterMotor.getPIDController();
        controller.setP(ShooterConstants.kP);
        controller.setI(ShooterConstants.kI);
        controller.setD(ShooterConstants.kD);
        controller.setFF(ShooterConstants.kFeedForward);

        //controller.setFeedbackDevice(shooterEncoder);

        // Setup PID output limits
        // controller.setOutputRange(
        //     ShooterConstants.kMinOutput,
        //     ShooterConstants.kMaxOutput
        // );
        /*
        controller.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        controller.setSmartMotionMaxVelocity(
            ShooterConstants.kMaxSpeedRotationsPerSecond * 60,
            0
        );
        controller.setSmartMotionMaxAccel(
            ShooterConstants.kMaxSpeedRotationsPerSecondSquared * 60,
            0
        );
        controller.setSmartMotionAllowedClosedLoopError(
            ShooterConstants.kMaxOutputError,
            0
        );
        */

        breakSensor = new DigitalInput(ShooterConstants.BREAK_SENSOR_PORT);

        targetRPM = 0;

        initShuffleboard();
    }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putBoolean("Is Note Fed?", isProjectileFed());
    // }

    public void setShooterVelocity(double speed) {
        targetRPM = speed;
        // if(speed == 0)  {
        //     setShooterRaw(0);
        //     return;
        // }
        controller.setReference(speed / 2, ControlType.kVelocity);
    }

    public void setTargetRPM(double target) {
        targetRPM = target;
    }

    public void runToTargetRPM() {
        setShooterVelocity(targetRPM);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isAtTarget(double target) {
        return Math.abs(shooterEncoder.getVelocity() - target) < ShooterConstants.kMaxOutputError;
    }

    public boolean isAtTarget() {
        return Math.abs(shooterEncoder.getVelocity() - targetRPM) < ShooterConstants.kMaxOutputError;
    }

    /**
     * Gets the speed of the shooter.
     *
     * @return The speed of the shooter
     */
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity() * 2;
    }

    /**
     * Gets the voltage supplied to the shooter.
     *
     * @return The bus voltage to the shooter
     */
    public double getShooterVoltage() {
        return shooterMotor.getBusVoltage();
    }

    /**
     * Stops supplying power to the shooter.
     */
    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    /**
     * Sets the speed of the feeder.
     *
     * @param speed The speed to set the feeder to
     */
    public void setFeederSpeed(double speed) {
        // Clamps the value on [-1, 1]
        //speed = Math.max(-1, Math.min(1, speed));

        feederMotor.set(speed);
    }

    public void setShooterRaw(double speed) {
        //speed = Math.max(-1, Math.min(1, speed));

        shooterMotor.set(speed);
    }

    /**
     * Stops the feeder
     */
    public void stopFeeder() {
        feederMotor.set(0);
    }

    public boolean isProjectileFed() {
        return breakSensor.get();
    }

    public double getShooterTemp() {
        return shooterMotor.getMotorTemperature();
    }

    public double getAmpShooterRPM() {
        return this.ampShooterRPM.getDouble(1.0);
    }

    private void initShuffleboard() {
        if(!Constants.DEBUG_MODE_ACTIVE) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.SHOOTER);

        ShuffleboardLayout persianPositions = moduleData.getLayout("Persian Positions", BuiltInLayouts.kList);

        persianPositions.addBoolean("Line Breaker", () -> breakSensor.get());
         persianPositions.addBoolean("At Target Speed", this::isAtTarget);

        persianPositions.addDouble("Shooter Velocity", this::getShooterVelocity);

        persianPositions.addDouble("Feeder Position", () -> feederEncoder.getPosition());

        persianPositions.addDouble("Feeder Velocity", () -> feederEncoder.getVelocity());

        persianPositions.withSize(2, 4);

        this.ampShooterRPM = moduleData
        .add("Amp Target RPM", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 4000))
        .getEntry();
    }
}
