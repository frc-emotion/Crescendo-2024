package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
    public static class IntakeIOInputs implements LoggableInputs {
        public boolean isDown = false;
        public boolean beamState = false;
        public double pivotPos = 0.0;
        public double pivotDeg = 0.0;
        public double pivotTarget = 0.0;
        public boolean atTarget = true;

        @Override
        public void toLog(LogTable table) {
            table.put("IsDown", isDown);
            table.put("BeamState", beamState);
            table.put("PivotPos", pivotPos);
            table.put("PivotDeg", pivotDeg);
            table.put("PivotTarget", pivotTarget);
            table.put("AtTarget", atTarget);
        }

        @Override
        public void fromLog(LogTable table) {
            isDown = table.get("IsDown", isDown);
            beamState = table.get("BeamState", beamState);
            pivotPos = table.get("PivotPos", pivotPos);
            pivotDeg = table.get("PivotDeg", pivotDeg);
            pivotTarget = table.get("PivotTarget", pivotTarget);
            atTarget = table.get("AtTarget", atTarget);
        }
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setDriveSpeed(double speed) {}
    public default void setPivotTarget(double target) {}
    public default void setPivotSpeed(double speed) {}
    public default void stopPivot() {}
    public default void stopDrive() {}
    public default void stop() {}
    public default void toggleIntake() {}
    public default void setIntakeDownState(boolean isDown) {}
    public default void resetPivotPos(double pos) {}
    public default void updateConstants(double kP, double kI, double kD, double maxSpeed, double maxAccel) {}
    public default void goToSetpoint() {}
}
