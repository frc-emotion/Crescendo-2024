package frc.robot.subsystems.pivot;

import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotDataMode;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;
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
    public static final PivotIOInputs inputs = new PivotIOInputs();

    private int index = 0;

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
        // layout.addDouble("Pivot Current", this::getCurrent);
        layout.addDouble("Pivot Preset Index", this::getIndex);
        layout.addNumber("Current Preset", this::getPreset);

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
     * Get the current index of Pivot
     * @return current index of Pivot
     */
    public int getIndex() {
        return index;
    }

    /**
     * Increment the index value
     */
    public void addIndex() {
        if (Constants.PivotConstants.PIVOT_POSITIONS.length - 1 != index) {
            index++;
        }
    }

    /**
     * Decrement the index value
     */
    public void subtractIndex() {
        if (index != 0) {
            index--;
        }
    }

    /**
     * Set the index value
     * @param i Value to set index to
     */
    public void setIndex(int i) {
        index = i;
    }

    /**
     * Move Pivot to preset target value
     */
    public void goToPreset() {
        setRev(getPreset());
    }

    /**
     * Get Pivot preset target value from PivotConstants
     * @return Pivot preset value
     */
    public double getPreset() {
        return Constants.PivotConstants.PIVOT_POSITIONS[index];
    }

    /**
     * Get Pivot preset target value from PivotConstants
     * @param ind Index of PivotConstants value to return
     * @return Pivot preset value from index ind
     */
    public double getPreset(int ind) {
        return Constants.PivotConstants.PIVOT_POSITIONS[ind];
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

    /**
     * Check if the current pivot angle would allow for a successful handoff
     * @return true if successful handoff is possible
     */
    public boolean isHandoffOk() {
        return this.getDegrees() - PivotConstants.kHANDOFF_ANGLE <= PivotConstants.kMAX_ANGLE_ERROR;
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
    public void setRev(double target) {
        // double target = rev;
        // if (rev < Constants.PivotConstants.PIVOT_MIN_REVOLUTION) {
        // target = 0;
        // }
        // if (rev > Constants.PivotConstants.PIVOT_MAX_REVOLUTION) {
        // target = Constants.PivotConstants.PIVOT_MAX_REVOLUTION;
        // }
        io.setTarget(target / Constants.PivotConstants.kConversionFactor);
    }

    /**
     * Move Pivot to handoff position
     */
    public void goToHandoff() {
        this.setRev(PivotConstants.kHANDOFF_ANGLE);
    }

    /**
     * Check if Pivot is at target position
     * @return true if Pivot is at target position
     */
    public boolean isAtTarget() {
        return Math.abs(getDegrees() - getPreset()) < PivotConstants.MAX_ERROR;
    }

    /**
     * Check if Pivot is at target position
     * @param degrees Custom target position to check
     * @return true if Pivot is at target position
     */
    public boolean isAtTarget(double degrees) {
        return Math.abs(getDegrees() - degrees) < PivotConstants.MAX_ERROR;
    }

}