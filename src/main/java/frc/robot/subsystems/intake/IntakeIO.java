package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO Layer for the Intake Subsystem. DOES NOT MANAGE GEAR RATIOS.
 */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        /** Whether or not the intake is deployed. */
        public boolean isDown = false;

        /** The current state of the beam break. */
        public boolean beamState = false;

        /** The current positon of the pivot in rotations. */
        public double pivotPos = 0.0;

        /** The current position of the pivot in degrees. */
        public double pivotDeg = 0.0;

        /** The target position of the pivot in rotations. */
        public double pivotTarget = 0.0;

        /** Whether or not the pivot is at the desired target. */
        public boolean isPivotAtTarget = true;

        /** The current speed of the intake drive in rotations per minute. */
        public double driveSpeed = 0.0;

        /** The current position of the intake drive motor in rotations */
        public double drivePos = 0.0;

        /** The target speed of the intake drive in rotations per minute. Only used in PID control mode.  */
        public double targetDriveSpeed = 0.0;

        /** Whether or not the intake drive is at the desired target. Only used in PID control mode. */
        public boolean isDriveAtTarget = true;
    }

    /** Updates the {@link IntakeIOInputs} with the most recent data. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Sends a raw speed from [-1, 1] to the Intake Drive Motor. */
    public default void setDriveRaw(double speed) {}

    /** Sets a target velocity for the Intake Drive Motor in rotations per minute. Uses PID Control. */
    public default void setDriveVelocity(double target) {}

    /** Sends a specific voltage directly to the Intake Drive Motor in volts. */
    public default void setDriveVoltage(double volts) {}

    /** Sets a target position for the Intake Pivot Motor in rotations. Uses PID Control. Requires {@link #goToPivotSetpoint()} in order to actually move the intake pivot.*/
    public default void setPivotTarget(double target) {}

    /** Sends a raw speed from [-1, 1] directly to the Intake Pivot Motor. */
    public default void setPivotRaw(double speed) {}

    /** Stops the Intake Pivot Motor */
    public default void stopPivot() {}

    /** Stops the Intake Drive Motor */
    public default void stopDrive() {}

    /** Stops the entire Intake Subsystem */
    public default void stop() {}
 
    /** Toggles the current Intake state. Does not actually move the intake. */
    public default void toggleIntake() {}

    /** Sets the current state of the Intake. Does not actually move the intake. */
    public default void setIntakeDownState(boolean isDown) {}
    
    /** Resets the Intake Pivot Encoder position to a specific position */
    public default void resetPivotPos(double pos) {}
    
    /** Updates all the constants for the Intake Subsystem. */
    public default void updateConstants(double kP, double kI, double kD, double maxSpeed, double maxAccel, double kP_Drive, double kI_Drive, double kD_Drive) {}

    /** Calculates and sendss motor inputs in order to travel to the desired setpoint. */
    public default void goToPivotSetpoint() {}
}
