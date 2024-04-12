package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs{
        public double velocity = 0.0;
        public double targetVelocity = 0.0;
        public boolean isAtTarget = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setShooterVelocity(double rpm) {}
    public default void setRaw(double speed) {}
    public default void updateConstants(double kP, double kI, double kD, double kFF) {}
    public default void stop() {}
}
