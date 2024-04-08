package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.RobotDataMode;
import frc.robot.util.TabManager.SubsystemTab;

public class SendableNumber {
    private ShuffleboardTab tab;
    private double defaultValue;

    private double lastValue;

    private GenericEntry shuffleboardEntry;

    private Supplier<Boolean> shouldRetrieveData;

    private boolean isEditable;

    public SendableNumber(SubsystemTab tab, String entryName) {
        this(tab, entryName, 0);
    }

    public SendableNumber(SubsystemTab tab, String entryName, double defaultValue) {
        this(tab, entryName, defaultValue, () -> Constants.ROBOT_DATA_MODE != RobotDataMode.MATCH);
    }

    public SendableNumber(SubsystemTab tab, String entryName, double defaultValue, Supplier<Boolean> shouldRetrieveDatSupplier) {
        this.defaultValue = defaultValue;
        lastValue = defaultValue;

        this.tab = TabManager.getInstance().accessTab(tab);
        shuffleboardEntry = this.tab.add(entryName, defaultValue).getEntry();

        shouldRetrieveData = shouldRetrieveDatSupplier;
    }
    
    public double getDefaultValue() {
        return defaultValue;
    }

    public void setDefaultValue(double value) {
        defaultValue = value;
        if(shouldRetrieveData.get()) {
            shuffleboardEntry.setDefaultDouble(defaultValue);
        } else {
            shuffleboardEntry.unpublish();
        }
    }

    public double get() {
        if(shouldRetrieveData.get()) {
            return lastValue = shuffleboardEntry.getDouble(defaultValue);
        } else {
            shuffleboardEntry.unpublish();
            return lastValue;
        }
    }

    public void set(double value) {
        if(shouldRetrieveData.get()) {
            shuffleboardEntry.setDouble(value);
        } else {
            shuffleboardEntry.unpublish();
        }
    }

    public boolean hasChanged() {
        double temp = lastValue;
        return get() != temp;
    }

    public void setEditAccess(boolean isEditable) {
        this.isEditable = isEditable;
    }

    public void readOnly() {
        isEditable = false;
    }

    public boolean isEditable() {
        return isEditable;
    }
}
