package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double pivotPos = 0.0;
        public double pivotAngularSpeed = 0.0;
        public double target = 0.0;
        public boolean atTargetAngle = true;

    }
    
    public default void updateInputs(PivotIOInputs inputs) {}
    public default void set(double speed) {}
    public default void setTarget(double target) {}
    public default void resetPosition(double position) {}
    public default void updatePID(double kP, double kI, double kD) {}
    public default void stop() {}
}
