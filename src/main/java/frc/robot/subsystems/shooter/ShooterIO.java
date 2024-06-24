package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** The IO Layer for the Shooter Subsystem. THIS IO LAYER DOES HANDLE GEAR RATIOS. */
public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        /** The current velocity of the Shooter in rotations per minute. */
        public double velocity = 0.0;
        
        /** The target velocity of the Shooter in rotations per minute. */
        public double targetVelocity = 0.0;

        /** Whether or not the Shooter is at the desired target. */
        public boolean isAtTarget = false;
    }

    /**
     * Updates the {@link ShooterIOInputs} with the current data.
     */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Sets the target shooter velocity using PID.
     * @param rpm   The target velocity in rotations per minute.
     */
    public default void setShooterVelocity(double rpm) {}

    /**
     * Sets the speed of the shooter motor directly.
     * @param speed The speed to set the shooter motor to. Range of [-1, 1].
     */
    public default void setRaw(double speed) {}

    /** Updates the Shooter PID constants */
    public default void updateConstants(double kP, double kI, double kD, double kFF) {}

    /** Immediately stops the entire Shooter Subsystem */
    public default void stop() {}
}
