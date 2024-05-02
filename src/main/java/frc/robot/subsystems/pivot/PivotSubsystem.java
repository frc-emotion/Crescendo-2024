package frc.robot.subsystems.pivot;

import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotDataMode;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Pivot Subsystem
 */
public class PivotSubsystem extends SubsystemBase {
    SendableNumber Pivot_kP = new SendableNumber(SubsystemTab.PIVOT, "Pivot kP", PivotConstants.PIVOT_KP); 
    SendableNumber Pivot_kI = new SendableNumber(SubsystemTab.PIVOT, "Pivot kI", PivotConstants.PIVOT_KI);
    SendableNumber Pivot_kD = new SendableNumber(SubsystemTab.PIVOT, "Pivot kD", PivotConstants.PIVOT_KD);


    private PivotIO io;
    public static final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private boolean turretMode = false;

    public PivotSubsystem(PivotIO io) {
        this.io = io;
        resetPosition(65.0);
        initShuffleboard();
    }

    /**
     * Constructs a new Pivot Subsystem instance
     */
    public PivotSubsystem() {
        this(new PivotIOSparkMax());
    }

    public void updateConstants() {
        io.updatePID(Pivot_kP.get(), Pivot_kI.get(), Pivot_kD.get());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    /**
     * Check if pivot is in turret mode
     * @return true if turret mode is enabled
     */
    public boolean turretMode() {
        return turretMode;
    }

    /**
     * Toggle pivot's turret mode state
     */
    public void toggleTurret() {
        turretMode = !turretMode;
    }

    /**
     * Initialize Pivot Shuffleboard info
     */
    private void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == RobotDataMode.MATCH) return;

        ShuffleboardTab tab = TabManager.getInstance().accessTab(SubsystemTab.PIVOT);
        ShuffleboardLayout layout = tab.getLayout("Persian Positions", BuiltInLayouts.kList);

        layout.addDouble("Pivot Position Revolutions", this::getRev);
        layout.addDouble("Pivot Position Degrees", this::getDegrees);

        layout.withSize(2, 4);
    }

    /**
     * Reset position of pivot
     * @param offset
     */
    public void resetPosition(double offset) {
        io.resetPosition(offset / PivotConstants.kConversionFactor);
    }

    /**
     * Set the speed of Pivot Motor
     * @param speed Speed to set Pivot Motor (-1 to 1)
     */
    public void setSpeed(double speed) {
        io.set(speed);
    }

    /**
     * Reset position of Pivot
     */
    public void calibrate() {
        resetPosition(0);
    }

    /**
     * Get relative encoder position
     * @return Relative encoder position adjusted for kConversionFactor
     */
    public double getRev() {
        return inputs.pivotPos * PivotConstants.kConversionFactor / 360;
    }

    /**
     * Get relative encoder position in degrees
     * @return Relative encoder position in degrees
     */
    public double getDegrees() {
        return getRev() * 360.0;
    }

    public double getAngularSpeed() {
        return inputs.pivotAngularSpeed;
    }

    /**
     * Check if the current pivot angle would allow for a successful handoff
     * @return true if successful handoff is possible
     */
    public boolean isHandoffOk() {
        return isAtTarget(PivotConstants.kHANDOFF_ANGLE);
    }

    /**
     * Stop the Pivot motor
     */
    public void stop() {
        io.stop();
    }

    /**
     * Move Pivot to provided target position
     * @param target Position to move Pivot to
     */
    public void setDegrees(double target) {
        io.setTarget(target / Constants.PivotConstants.kConversionFactor);
    }

    /**
     * Move Pivot to handoff position
     */
    public void goToHandoff() {
        this.setDegrees(PivotConstants.kHANDOFF_ANGLE);
    }

    /**
     * Check if Pivot is at target position
     * @param degrees Custom target position to check
     * @return true if Pivot is at target position
     */
    public boolean isAtTarget(double degrees) {
        return Math.abs(getDegrees() - degrees) < PivotConstants.MAX_ERROR;
    }

    public boolean isAtTarget() {
        return inputs.atTargetAngle;
    }

}