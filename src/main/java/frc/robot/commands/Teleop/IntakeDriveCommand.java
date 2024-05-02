package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Runs the intake manually. Teleop only version. Currently only used
 * to reverse the intake.
 */
public class IntakeDriveCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> forwardFunc, reverseFunc;

    public IntakeDriveCommand(IntakeSubsystem intake,
            Supplier<Boolean> forwardFunc,
            Supplier<Boolean> reverseFunc) {

        this.intakeSubsystem = intake;
        this.forwardFunc = forwardFunc;
        this.reverseFunc = reverseFunc;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        /*
         * if (leftAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
         * intakeSubsystem.revSimplePivot();
         * } else if (rightAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
         * intakeSubsystem.simplePivot();
         * } else {
         * intakeSubsystem.pivotStop();
         * }
         */

        if (forwardFunc.get() && intakeSubsystem.getBeamState()) {
            intakeSubsystem.intakeForward();
        } else if (reverseFunc.get()) {
            intakeSubsystem.intakeReverse();
        } else {
            intakeSubsystem.stopDrive();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
