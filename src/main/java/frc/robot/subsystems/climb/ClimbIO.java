package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs{
        /** The current position of the left climb arm in rotations. */
        public double leftPos = 0.0;

        /** The current position of the right climb arm in rotations. */
        public double rightPos = 0.0;

        /** The current position of the highest climb arm in rotations. */
        public double highestPos = 0.0;

        /** Whether or not the climb arms are aligned */
        public boolean isClimbAligned = true;
    }

    /** Updates the {@link ClimbIOInputs} with the most recent data. */
    public default void updateInputs(ClimbIOInputs inputs) {}

    /** Sends a raw speed from [-1, 1] to the Climb motors */
    public default void set(double speed) {}

    /** Resets the Climb encoders. */
    public default void reset() {}

    /** Stops the Climb Subsystem. */
    public default void stop() {}
}