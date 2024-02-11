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
    }

    @Override
    public void execute() {
        if (bButton.get()) {
            pivotSubsystem.calibrate();
        } else if (Math.abs(yJoystick.get()) > Constants.PivotConstants.TRIGGER_THRESHOLD) {

            if (!pivotSubsystem.getLimit() && yJoystick.get() > 0.0) {
                pivotSubsystem.calibrate();
                return;
            }

            if (pivotSubsystem.getRev() < Constants.PivotConstants.PIVOT_ZERO_THRESHOLD && yJoystick.get() > 0.0) {
                pivotSubsystem.setSpeed(yJoystick.get() * Constants.PivotConstants.PIVOT_ZERO_SPEED);
                return;
            }

            if (pivotSubsystem.getRev() > Constants.PivotConstants.PIVOT_MAX_REVOLUTION && yJoystick.get() < 0.0) {
                return;
            }
            pivotSubsystem.setSpeed(yJoystick.get() * Constants.PivotConstants.PIVOT_TELEOP_SPEED);

        } else if (leftStickButton.get()) {
            pivotSubsystem.setRev(0);
        } else {

            if (dPadDown.get()) {
                pivotSubsystem.subtractIndex();
            } else if (dPadUp.get()) {
                pivotSubsystem.addIndex();
            } else {
                pivotSubsystem.goToPreset();
            }

        }
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
