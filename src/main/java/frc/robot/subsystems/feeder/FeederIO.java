package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double velocity = 0.0;
        public double targetVelocity = 0.0;
        public boolean isAtTarget = false;
        public boolean isBeamBroken = false;
    }
    public default void updateInputs(FeederIOInputsAutoLogged inputs) {}
    public default void set(double speed) {}
    public default void setVelocity(double velocity) {}
    public default void updateConstants(double kP, double kI, double kD, double kFF) {}
    public default void stop() {}
}
