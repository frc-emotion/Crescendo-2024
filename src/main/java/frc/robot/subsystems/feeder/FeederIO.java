package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** The IO Layer for the Feeder Subsystem. */
public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs{
        /** The current velocity of the feeder in rotations per minute. */
        public double velocity = 0.0;

        /** The target velcocity of the feeder in rotations per minute. Only used in PID control mode. */
        public double targetVelocity = 0.0;

        /** Whether or not the Feeder is currently at the target speed. Only used in PID control mode. */
        public boolean isAtTarget = false;
        
        /** The current state of the beam break. */
        public boolean isBeamBroken = false;
    }
    
    /** Updates the {@link FeederIOInputs} to the most recent data. */
    public default void updateInputs(FeederIOInputs inputs) {}

    /** Sets the Feeder Speed using raw input from [-1, 1]. */
    public default void set(double speed) {}

    /** Sets the velocity of the Feeder in rotations per minute. */
    public default void setVelocity(double velocity) {}

    /** Sends a voltage to the motor in volts. */
    public default void setVoltage(double voltage) {}

    /** Updates the Feeder Subsystem constants. Mainly PID. */
    public default void updateConstants(double kP, double kI, double kD, double kFF) {}

    /** Stops the Feeder Subsystem. */
    public default void stop() {}
}
