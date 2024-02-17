package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOStates {
        public double positionMeters = 0.0;
        public double speedMetersPerSecond = 0.0;

    }
    
    public default void updateInputs(ModuleIOStates input) {}
}