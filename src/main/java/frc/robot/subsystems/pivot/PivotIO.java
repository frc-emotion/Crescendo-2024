package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIO {
    public static class PivotIOInputs implements LoggableInputs {
        public double pivotPos = 0.0;
        public double pivotAngularSpeed = 0.0;
        public double target = 0.0;
        public boolean atTargetAngle = true;

        @Override
        public void toLog(LogTable table) {
            table.put("PivotPos", pivotPos);
            table.put("PivotAngularSpeed", pivotAngularSpeed);
            table.put("TargetPos", target);
            table.put("AtTargetAngle", atTargetAngle);
        }

        @Override
        public void fromLog(LogTable table) {
            pivotPos = table.get("PivotPos", pivotPos);
            pivotAngularSpeed = table.get("PivotAngularSpeed", pivotAngularSpeed);
            target = table.get("TargetPos", target);
            atTargetAngle = table.get("AtTargetAngle", atTargetAngle);
        }
    }
    public default void updateInputs(PivotIOInputs inputs) {}
    public default void set(double speed) {}
    public default void setTarget(double target) {}
    public default void resetPosition(double position) {}
    public default void updatePID(double kP, double kI, double kD) {}
    public default void stop() {}
}
