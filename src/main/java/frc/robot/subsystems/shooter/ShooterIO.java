package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
    public class ShooterIOInputs implements LoggableInputs {
        public double velocity = 0.0;
        public double targetVelocity = 0.0;
        public boolean isAtTarget = false;

        @Override
        public void toLog(LogTable table) {
            table.put("Velocity", velocity);
            table.put("TargetVelocity", targetVelocity);
            table.put("IsAtTarget", isAtTarget);
        }

        @Override
        public void fromLog(LogTable table) {
            velocity = table.get("Velocity", velocity);
            targetVelocity = table.get("TargetVelocity", targetVelocity);
            isAtTarget = table.get("IsAtTarget", isAtTarget);
        }

    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setShooterVelocity(double rpm) {}
    public default void setRaw(double speed) {}
    public default void updateConstants(double kP, double kI, double kD, double kFF) {}
    public default void stop() {}
}
