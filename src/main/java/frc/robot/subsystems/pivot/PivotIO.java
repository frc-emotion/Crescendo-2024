package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        /** The current position of the pivot in rotations. */
        public double pivotPos = 0.0;

        /** The current speed of the pivot in rotations per minute. */
        public double pivotAngularSpeed = 0.0;

        /** The target position of the pivot in rotations. */
        public double target = 0.0;

        /** Whether or not the pivot is at the desired angle. */
        public boolean atTargetAngle = true;

    }
    /** Updates the {@link PivotIOInputs} with the most recent data. */
    public default void updateInputs(PivotIOInputs inputs) {}

    /** Sets a raw speed directly to the Pivot motor. Range of [-1, 1]. */
    public default void set(double speed) {}

    /** Sets the position target of the Pivot in rotations. Uses PID. */
    public default void setTarget(double target) {}
    public default void resetPosition(double position) {}

    /** Updates the Pivot PID constants. */
    public default void updatePID(double kP, double kI, double kD) {}

    /** Immediately stops the Pivot Subsytem. */
    public default void stop() {}
}
