package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Shooter Subsystem
 */
public class ShooterSubsystem extends SubsystemBase {
    SendableNumber Shooter_kP = new SendableNumber(SubsystemTab.SHOOTER, "Shooter kP", ShooterConstants.SHOOTER_kP);
    SendableNumber Shooter_kI = new SendableNumber(SubsystemTab.SHOOTER, "Shooter kI", ShooterConstants.SHOOTER_kI);
    SendableNumber Shooter_kD = new SendableNumber(SubsystemTab.SHOOTER, "Shooter kD", ShooterConstants.SHOOTER_kD);
    SendableNumber Shooter_kFF = new SendableNumber(SubsystemTab.SHOOTER, "Shooter kFF", ShooterConstants.SHOOTER_kFeedForward);
    SendableNumber AmpRPM = new SendableNumber(SubsystemTab.SHOOTER, "Shooter Amp RPM", ShooterConstants.AmpRPM);
    SendableNumber SpeakerRPM = new SendableNumber(SubsystemTab.SHOOTER, "Shooter Speaker RPM", ShooterConstants.SHOOTER_SPEED_RPM);

    private ShooterIO io;
    private static final ShooterIOInputs inputs = new ShooterIOInputs();
    /**
     * Construct a new instance of Shooter Subsystem
     */
    public ShooterSubsystem() {
        this(new ShooterIOSparkMax());
    }

    public ShooterSubsystem(ShooterIO io ) {
        this.io = io;
        initShuffleboard();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /**
     * Update PID values for Shooter
     */
    public void updatePID() {
        io.updateConstants(
            Shooter_kP.get(),
            Shooter_kI.get(),
            Shooter_kD.get(),
            Shooter_kFF.get()
        );
    }

    /**
     * Set the shooter velocity
     * @param speed Speed to set the shooter to
     */
    public void setShooterVelocity(double speed) {
        io.setShooterVelocity(speed / 2);
    }
    
    /**
     * Check if the shooter is at the target RPM
     * @param target Custom target RPM to check
     * @return true if Shooter has reached target RPM
     */
    public boolean isAtTarget(double target) {
        return (
            Math.abs(inputs.velocity - target) <
            ShooterConstants.kMaxOutputError
        );
    }

    /**
     * Check if the shooter is at the target RPM
     * @return true if Shooter has reached target RPM
     */
    public boolean isAtTarget() {
        return inputs.isAtTarget;
    }

    /**
     * Gets the speed of the shooter.
     *
     * @return The speed of the shooter
     */
    public double getShooterVelocity() {
        return inputs.velocity;
    }

    /**
     * Gets the voltage supplied to the shooter.
     *
     * @return The bus voltage to the shooter
     */
    public double getShooterVoltage() {
        return inputs.voltage;
    }

    /**
     * Stops supplying power to the shooter.
     */
    public void stopShooter() {
        io.stop();
    }
    /**
     * Sets the speed of the shooter.
     *
     * @param speed The speed to set the shooter to
     */
    public void setShooterRaw(double speed) {
        io.setRaw(speed);
    }

    /**
     * Get the current temperature of the Shooter
     * @return Shooter Motor temperature in Celsius
     */
    public double getShooterTemp() {
        return inputs.temp;
    }

    public double getAmpRPM() {
        return AmpRPM.get();
    }

    public double getSpeakerRPM() {
        return SpeakerRPM.get();
    }

    public double getTargetVelocity() {
        return inputs.targetVelocity;
    }

    public void runAmpRPM() {
        setShooterVelocity(getAmpRPM());
    }

    public void runSpeakerRPM() {
        setShooterVelocity(getSpeakerRPM());
    }

    /**
     * Initialize Shuffleboard info for Shooter
     */
    private void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == Constants.RobotDataMode.MATCH) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.SHOOTER);

        ShuffleboardLayout persianPositions = moduleData.getLayout(
            "Shooter Info",
            BuiltInLayouts.kList
        );

        persianPositions.addBoolean("At Target Speed", this::isAtTarget);

        persianPositions.addDouble(
            "Shooter Velocity",
            this::getShooterVelocity
        );

        persianPositions.addDouble(
            "Target RPM",
            () -> getTargetVelocity()
        );

        persianPositions.withSize(2, 4);
    }
}
