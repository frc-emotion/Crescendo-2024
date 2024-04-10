package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ClimbIO {
    public static class ClimbIOInputs implements LoggableInputs {
        public double leftPos = 0.0;
        public double rightPos = 0.0;
        public double highestPos = 0.0;
        public boolean isClimbAligned = true;
        public double current = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("LeftPos", leftPos);
            table.put("RightPos", rightPos);
            table.put("HighestPos", highestPos);
            table.put("IsClimbAligned", isClimbAligned);
            table.put("Current", current);
        }

        @Override
        public void fromLog(LogTable table) {
            leftPos = table.get("LeftPos", leftPos);
            rightPos = table.get("RightPos", rightPos);
            highestPos = table.get("HighestPos", highestPos);
            isClimbAligned = table.get("IsClimbAligned", isClimbAligned);
            current = table.get("Current", current);
        }
    }

    public default void updateInputs(ClimbIOInputs inputs) {}
    public default void set(double speed) {}
    public default void reset() {}
    public default void stop() {}
}