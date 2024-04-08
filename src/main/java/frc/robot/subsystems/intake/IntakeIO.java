package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
    public static class IntakeIOInputs implements LoggableInputs {
        public boolean isDown = false;
        public boolean beamState = false;
        public double pivotPos = 0.0;
        public double pivotDeg = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("IsDown", isDown);
            table.put("BeamState", beamState);
            table.put("PivotPos", pivotPos);
            table.put("PivotDeg", pivotDeg);
        }

        @Override
        public void fromLog(LogTable table) {
            isDown = table.get("IsDown", isDown);
            beamState = table.get("BeamState", beamState);
            pivotPos = table.get("PivotPos", pivotPos);
            pivotDeg = table.get("PivotDeg", pivotDeg);
        }
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setDrive(double speed) {}
    public default void setPivot(double speed) {}
    public default void stopPivot() {}
    public default void stopDrive() {}
    public default void stop() {}
    public default void toggleIntake() {}
    public default void setIntakeDownState(boolean isDown) {}
    public default void resetPivotPos(double pos) {}
}
