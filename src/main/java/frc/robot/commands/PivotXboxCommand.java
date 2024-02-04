package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotXboxCommand extends Command {
    private final PivotSubsystem pivotSubsystem;

    private final Supplier<Double> yJoystick, dPad;
    private final Supplier<Boolean> xButton, yButton, leftStickButton;

    public PivotXboxCommand(PivotSubsystem pivotSubsystem, Supplier<Double> yJoystick,
            Supplier<Boolean> xButton,
            Supplier<Boolean> yButton,
            Supplier<Boolean> leftStickButton,
            Supplier<Double> dPad) {

        this.yJoystick = yJoystick;
        this.xButton = xButton;
        this.yButton = yButton;
        this.leftStickButton = leftStickButton;
        this.dPad = dPad;
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        if (xButton.get()) {
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
            switch (dPad.get().intValue()) {
                // TODO: Needs to be updated thru testing
                case 0:
                    pivotSubsystem.setAgainst();
                    break;
                case 90:
                    pivotSubsystem.setLine();
                    break;
                case 180:
                    pivotSubsystem.setWheel();
                    break;
                case 270:
                    pivotSubsystem.setTrench();
                    break;
                default:
                    pivotSubsystem.stop();
                    break;
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
