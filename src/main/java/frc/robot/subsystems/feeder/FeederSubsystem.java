package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

public class FeederSubsystem extends SubsystemBase {
    SendableNumber Feeder_kP = new SendableNumber(SubsystemTab.SHOOTER, "Feeder kP", ShooterConstants.FEEDER_kP);
    SendableNumber Feeder_kI = new SendableNumber(SubsystemTab.SHOOTER, "Feeder kI", ShooterConstants.FEEDER_kI);
    SendableNumber Feeder_kD = new SendableNumber(SubsystemTab.SHOOTER, "Feeder kD", ShooterConstants.FEEDER_kD);
    SendableNumber Feeder_kFF = new SendableNumber(SubsystemTab.SHOOTER, "Feeder kFF", ShooterConstants.FEEDER_kFF);
    SendableNumber FeederSpeed = new SendableNumber(SubsystemTab.SHOOTER, "Feeder Target Speed", ShooterConstants.FEEDER_SPEED);

    private FeederIO io;
    private static final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    /**
     * Updates the inputs from the IO layer
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    /**
     * Updates the PID Constants in the IO layer using the SendableNumbers
     */
    public void updateConstants() {
        io.updateConstants(
            Feeder_kP.get(),
            Feeder_kI.get(),
            Feeder_kD.get(),
            Feeder_kFF.get()
        );
    }

    public void set(double speed) {
        io.set(speed);
    }

    /**
     * Sets the feeder velocity using PID
     * @param velocity  The target velocity in RPM
     */
    public void setFeederVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.stop();
    }

    /**
     * Gets the target velocity for feeding
     * @return  The target feeding velocity in RPM
     */
    public double getFeedingVelocity() {
        return FeederSpeed.get();
    }

    /**
     * Gets the current velocity of the feeder from the IO layer inputs
     * @return  The current feeder velocity in RPM
     */
    public double getVelocity() {
        return inputs.velocity;
    }

    /**
     * Gets te current target velocity of the feeder from the IO layer inputs
     * @return The current target feeder velocity in RPM
     */
    public double getTargetVelocity() {
        return inputs.targetVelocity;
    }

    /**
     * Gives whether the feeder is currently at the target velocity
     * @return  Whether the feeder is currently at the target velocity
     */
    public boolean isAtTarget() {
        return inputs.isAtTarget;
    }

    /**
     * Gets the current state of the beam sensor form the IO layer inputs
     * @return Whether or not the beam is broken
     */
    public boolean getBeamState() {
        return inputs.isBeamBroken;
    }

    /** Initializes the Shuffleboard data for the feeder */
    public void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == Constants.RobotDataMode.MATCH) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.SHOOTER);

        ShuffleboardLayout persianPositions = moduleData.getLayout(
            "Feeder Info",
            BuiltInLayouts.kList
        );

        persianPositions.addBoolean("At Target Speed", this::isAtTarget);
        persianPositions.addBoolean("Is Projectile Loaded", this::getBeamState);

        persianPositions.addDouble(
            "Feeder Velocity",
            this::getVelocity
        );

        persianPositions.addDouble(
            "Target RPM",
            () -> getTargetVelocity()
        );

        persianPositions.withSize(2, 4);
    }
}
