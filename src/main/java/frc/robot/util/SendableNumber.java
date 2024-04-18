package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.RobotDataMode;
import frc.robot.util.TabManager.SubsystemTab;

public class SendableNumber {
    /** The Shuffleboard tab the Shuffleboard entry is shown and editable from */
    private ShuffleboardTab tab;

    /** The default value for the number */
    private double defaultValue;

    /** The last inputted value for the number */
    private double lastValue;

    /** The editable Shuffleboard number */
    private GenericEntry shuffleboardEntry;

    /** Whether or not the Shuffleboard entry should be accessed. */
    private Supplier<Boolean> shouldRetrieveData;

    /** Whether or not the edited Shuffleboard number should be used. Does not actually restrict editing.  */
    private boolean isEditable;

    /** Constructs a new SendableNumber with a given ShuffleboardTab and entry name. */
    public SendableNumber(SubsystemTab tab, String entryName) {
        this(tab, entryName, 0);
    }

    /** Constructs a new SendableNumber using a given ShuffleboardTab, entryName, and default value. */
    public SendableNumber(SubsystemTab tab, String entryName, double defaultValue) {
        this(tab, entryName, defaultValue, () -> Constants.ROBOT_DATA_MODE != RobotDataMode.MATCH);
    }
    
    /** Not really used. */
    public SendableNumber(SubsystemTab tab, String entryName, double defaultValue, Supplier<Boolean> shouldRetrieveDatSupplier) {
        this.defaultValue = defaultValue;
        lastValue = defaultValue;

        this.tab = TabManager.getInstance().accessTab(tab);
        shuffleboardEntry = this.tab.add(entryName, defaultValue).getEntry();

        shouldRetrieveData = shouldRetrieveDatSupplier;
    }
    
    /** Returns the default value */
    public double getDefaultValue() {
        return defaultValue;
    }

    /** Sets the default value */
    public void setDefaultValue(double value) {
        defaultValue = value;
        if(shouldRetrieveData.get()) {
            shuffleboardEntry.setDefaultDouble(defaultValue);
        } else {
            shuffleboardEntry.unpublish();
        }
    }

    /** Gets the current value from Shuffleboard entry if editable and able to access data. Otherwise,
     * it returns the last read value.
     */
    public double get() {
        if(shouldRetrieveData.get() && !isEditable) {
            return lastValue = shuffleboardEntry.getDouble(defaultValue);
        } else if(isEditable) {
            return lastValue;
        } else {
            shuffleboardEntry.unpublish();
            return lastValue;
        }
    }

    /**
     * Sets the current value of the Shuffleboard entry, or if the entry is unavailable, it unpublishes the entry.
     */
    public void set(double value) {
        lastValue = value;
        if(shouldRetrieveData.get()) {
            shuffleboardEntry.setDouble(value);
        } else {
            shuffleboardEntry.unpublish();
        }
    }

    /** Checks whether or not the current value is different from the last value */
    public boolean hasChanged() {
        double temp = lastValue;
        return get() != temp;
    }

    /** Sets the edit access permission */
    public void setEditAccess(boolean isEditable) {
        this.isEditable = isEditable;
    }
    
    /** Checks whether or not the Shuffleboard entry should be editable. */
    public boolean isEditable() {
        return isEditable;
    }
}
