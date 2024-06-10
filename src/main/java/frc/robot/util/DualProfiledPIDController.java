package frc.robot.util;

import java.util.Optional;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Dual Profiled PID for systems which may require it. Uses a safety system
 * in order to detect and manage if the system reaches an unsafe state during
 * operation.
 * 
 * Not sure if we actually need this but I made it anyway.
 * 
 * @author Peyton Slape
 */
public class DualProfiledPIDController {
    private ProfiledPIDController pidOne, pidTwo;

    private double maxSafetyError;
    
    private double targetOne, targetTwo;
    private final double defaultPosOne, defaultPosTwo;
    
    private SafetyMode safetyMode;

    /**
     * The level of safety to use. Warnings are printed to the console
     * in all modes except DISABLED. RETURN returns to the default position 
     * as defined in the constructor.
     */
    public enum SafetyMode {
        DISABLED,
        WARN_ONLY,
        STOP,
        RETURN
    }

    // Syncs both PID controllers essentially. Basically only used for error monitoring.
    public DualProfiledPIDController(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints, double defaultPos, double maxSafetyError, SafetyMode safetyMode) {
        this(kP, kI, kD, constraints, defaultPos, kP, kI, kD, constraints, defaultPos, maxSafetyError, safetyMode);
    }

    // May be a bit too many args. Somebody lmk if it is.
    public DualProfiledPIDController(
        double kP1, 
        double kI1, 
        double kD1, 
        TrapezoidProfile.Constraints constraintsOne, 
        double defaultPosOne,
        double kP2, 
        double kI2, 
        double kD2, 
        TrapezoidProfile.Constraints constraintsTwo,
        double defaultPosTwo,
        double maxSafetyError,
        SafetyMode safetyMode
    ) {
        pidOne = new ProfiledPIDController(kP1, kI1, kD1, constraintsOne);
        pidTwo = new ProfiledPIDController(kP2, kI2, kD2, constraintsTwo);
        this.maxSafetyError = maxSafetyError;
        this.defaultPosOne = defaultPosOne;
        this.defaultPosTwo = defaultPosTwo;
        this.safetyMode = safetyMode;
    }

    /**
     * Calculates the PID outputs, includes the safety override if it is enabled.
     */
    public double[] calculate(double measurementOne, double targetOne, double measurementTwo, double targetTwo)  {
        double[] results = new double[] {
            pidOne.calculate(measurementOne, targetOne),
            pidTwo.calculate(measurementTwo, targetTwo)
        };

        setTargets(targetOne, targetTwo);

        if(!withinSafetyLimits(measurementOne, measurementTwo)) {
            switch(safetyMode) {
                case STOP:
                    DriverStation.reportWarning("Dual PID Error Threshold Passed. Stopping mechanism.", null);
                    return new double[] {0, 0};
                case RETURN:
                    DriverStation.reportWarning("Dual PID Error Threshold Passed, Returning to default position.", null);
                    return calculate(measurementOne, measurementTwo, defaultPosOne, defaultPosTwo);
                case WARN_ONLY:
                    DriverStation.reportWarning("Dual PID Error Threshold Passed", null);
                    break;
                case DISABLED:
                    break;
            }
        }

        return results;
    }

    /**
     * Calculates PID based off of previous targets
     */
    public double[] calculate(double measurementOne, double measurementTwo) {
        return calculate(measurementOne, measurementTwo, targetOne, targetTwo);
    }

    /**
     * Checks if the system is within the max error as provided in the constructor.
     * Returns true if the safety mode is disabled. 
     */
    public boolean withinSafetyLimits(double measurementOne, double measurementTwo) {
        return (Math.abs(measurementOne - measurementTwo) < maxSafetyError) || !isSafetyEnabled();
    }

    /**
     * Sets the PID targets.
     */
    public void setTargets(double targetOne, double targetTwo) {
        this.targetOne = targetOne;
        this.targetTwo = targetTwo;
    }

    /**
     * Checks if the current safety mode is SafetyMode.DISABLED.
     * @return
     */
    public boolean isSafetyEnabled() {
        return !safetyMode.equals(SafetyMode.DISABLED);
    }
    
}
