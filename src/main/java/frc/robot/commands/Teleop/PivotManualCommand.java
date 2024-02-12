package frc.robot.commands.Teleop;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotManualCommand extends Command {
    private final PivotSubsystem pivotSubsystem;

    private final Supplier<Double> yJoystick;
    private final Supplier<Boolean> bButton, leftStickButton, dPadUp, dPadDown;

    private boolean mode; // true = manual, false = auto

    public PivotManualCommand(PivotSubsystem pivotSubsystem, 
            Supplier<Double> yJoystick,
            Supplier<Boolean> bButton,
            Supplier<Boolean> leftStickButton,
            Supplier<Boolean> dPadUp,
            Supplier<Boolean> dPadDown) {

        this.yJoystick = yJoystick;
        this.bButton = bButton;
        this.leftStickButton = leftStickButton;
        this.dPadUp = dPadUp;
        this.dPadDown = dPadDown;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
        mode = false;
    }

    @Override
    public void execute() {  
        if (bButton.get()) {    // If button is pressed, calibrates the pivot encoders
            pivotSubsystem.calibrate();
            
        } else if (Math.abs(yJoystick.get()) > Constants.PivotConstants.TRIGGER_THRESHOLD) {    // If manual input is detected, changes mode to manual mode.
            mode = true;
            
        } else if (leftStickButton.get()) { // If stick is depressed, turns off the motor
            mode = true;
            pivotSubsystem.setRev(0);

        } else {    // Automatic Control
            if (dPadDown.get()) { // If dpad down is pressed, sets to automatic mode and changes to the next lowest preset as target
                mode = false;
                pivotSubsystem.subtractIndex();

            } else if (dPadUp.get()) {  // if dpad up is pressed, sets to automatic mode and changes to next highest preset as target
                mode = false;
                pivotSubsystem.addIndex();
            }
        }

        if(mode) { 
            manualControl();
        } else {
            pivotSubsystem.goToPreset();
        }
    }

    /**
     * Manages the manual control mode for the 
     */
    private void manualControl() {
        double output;
         
        if(pivotSubsystem.getRev() >= Constants.PivotConstants.PIVOT_MAX_REVOLUTION) {   // If over or equal to maximum position, makes it so the pivot can only go down
            output = Clamp(yJoystick.get(), 0, -1); 

        } else if(pivotSubsystem.getRev() <= Constants.PivotConstants.PIVOT_MIN_REVOLUTION) {   // If under or equal to the minimum position, makes it so the pivot can only go up
            output = Clamp(yJoystick.get(), 1, 0);
            
        } else {    // If the pivot is in normal operation between max and min values
            output = yJoystick.get();
        }

        pivotSubsystem.setSpeed(output * Constants.PivotConstants.PIVOT_TELEOP_SPEED);
    }

    /**
     * Clamps the input value between the maximum and the minimum
     * 
     * @param in    The input value to check
     * @param max   The max value
     * @param min   The min value
     * @return      The clamped input
     */
    private double Clamp(double in, double max, double min) {
        return Math.max(min, Math.min(in, max));
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // TODO: Anything else need to go here?
    }

}