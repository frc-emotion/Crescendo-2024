package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotManualCommand extends Command {
    private final PivotSubsystem pivotSubsystem;

    private final Supplier<Double> yJoystick;
    private final Supplier<Boolean> leftStickButton, dPadUp, dPadDown;

    private boolean mode; // true = manual, false = auto

    public PivotManualCommand(PivotSubsystem pivotSubsystem, 
            Supplier<Double> yJoystick,
            Supplier<Boolean> leftStickButton,
            Supplier<Boolean> dPadUp,
            Supplier<Boolean> dPadDown) {

        this.yJoystick = yJoystick;
        this.leftStickButton = leftStickButton;
        this.dPadUp = dPadUp;
        this.dPadDown = dPadDown;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
        mode = false;
    }

    @Override
    public void execute() { 
        if (pivotSubsystem.getCalibration() && pivotSubsystem.getCurrent() > PivotConstants.CURRENT_SPIKE_THRESHOLD) {
            pivotSubsystem.endCalibrate();
            return;
        } else if (pivotSubsystem.getCalibration()) {
            return;
        }

        // if (Math.abs(yJoystick.get()) > Constants.PivotConstants.TRIGGER_THRESHOLD) {    // If manual input is detected, changes mode to manual mode.
        //     mode = true;
            
        // } else if (leftStickButton.get()) { // If stick is depressed, turns off the motor
        //     mode = true;
        //     pivotSubsystem.setRev(0);

        // } else {    // Automatic Control
        //     if (dPadDown.get()) { // If dpad down is pressed, sets to automatic mode and changes to the next lowest preset as target
        //         mode = false;
        //         pivotSubsystem.subtractIndex();

        //     } else if (dPadUp.get()) {  // if dpad up is pressed, sets to automatic mode and changes to next highest preset as target
        //         mode = false;
        //         pivotSubsystem.addIndex();
        //     }
        // }

        manualControl(); // remove before official use

        /*
        if(mode) { 
            manualControl();
        } else {
            pivotSubsystem.goToPreset();
        }
        */
        //}

    }

    /**
     * Manages the manual control mode for the 
     */
    private void manualControl() {
        double output = yJoystick.get();

        pivotSubsystem.setSpeed(output * Constants.PivotConstants.PIVOT_TELEOP_SPEED);
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
