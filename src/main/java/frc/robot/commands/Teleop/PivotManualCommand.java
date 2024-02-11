package frc.robot.commands.Teleop;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotManualCommand extends Command {
    private final PivotSubsystem pivotSubsystem;

    private final Supplier<Double> yJoystick;
    private final Supplier<Boolean> bButton, leftStickButton, dPadRight, dPadUp, dPadLeft, dPadDown;

    private final HashMap<Supplier<Boolean>, Double> preSetPositions;

    public PivotManualCommand(PivotSubsystem pivotSubsystem, 
            Supplier<Double> yJoystick,
            Supplier<Boolean> bButton,
            Supplier<Boolean> leftStickButton,
            Supplier<Boolean> dPadRight,
            Supplier<Boolean> dPadUp,
            Supplier<Boolean> dPadLeft,
            Supplier<Boolean> dPadDown) {

        this.yJoystick = yJoystick;
        this.bButton = bButton;
        this.leftStickButton = leftStickButton;
        this.dPadRight = dPadRight;
        this.dPadUp = dPadUp;
        this.dPadLeft = dPadLeft;
        this.dPadDown = dPadDown;
        this.pivotSubsystem = pivotSubsystem;

        this.preSetPositions = new HashMap<Supplier<Boolean>, Double>();
        this.preSetPositions.put(dPadRight, Constants.PivotConstants.placeholder);
        this.preSetPositions.put(dPadUp, Constants.PivotConstants.placeholder);
        this.preSetPositions.put(dPadLeft, Constants.PivotConstants.placeholder);
        this.preSetPositions.put(dPadDown, Constants.PivotConstants.placeholder);

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

            boolean picked = false;
            for (Supplier<Boolean> dPadPosition : preSetPositions.keySet()) {
                if (dPadPosition.get()) {
                    picked = true;
                    pivotSubsystem.setRev(preSetPositions.get(dPadPosition));
                }
            }

            if (!picked) {
                pivotSubsystem.stop();
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
