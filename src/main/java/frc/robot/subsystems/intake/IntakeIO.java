package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public boolean isDown = false;
        public boolean beamState = false;
        public double pivotPos = 0.0;
        public double pivotDeg = 0.0;
        public double pivotTarget = 0.0;
        public boolean isPivotAtTarget = true;

        public double driveSpeed = 0.0;
        public double targetDriveSpeed = 0.0;
        public boolean isDriveAtTarget = true;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setDriveRaw(double speed) {}
    public default void setDriveVelocity(double target) {}
    public default void setDriveVoltage(double volts) {}
    public default void setPivotTarget(double target) {}
    public default void setPivotRaw(double speed) {}
    public default void stopPivot() {}
    public default void stopDrive() {}
    public default void stop() {}
    public default void toggleIntake() {}
    public default void setIntakeDownState(boolean isDown) {}
    public default void resetPivotPos(double pos) {}
    public default void updateConstants(double kP, double kI, double kD, double maxSpeed, double maxAccel, double kP_Drive, double kI_Drive, double kD_Drive) {}
    public default void goToPivotSetpoint() {}
}
