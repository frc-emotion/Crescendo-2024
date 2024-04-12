package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs{
        public double leftPos = 0.0;
        public double rightPos = 0.0;
        public double highestPos = 0.0;
        public boolean isClimbAligned = true;
        public double current = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}
    public default void set(double speed) {}
    public default void reset() {}
    public default void stop() {}
}