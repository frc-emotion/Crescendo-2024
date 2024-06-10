package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualPivotCommand extends Command {
    private Supplier<Boolean> outSupplier, inSupplier;
    private IntakeSubsystem intakeSubsystem;

    public IntakeManualPivotCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> out, Supplier<Boolean> in) {
        this.intakeSubsystem = intakeSubsystem;
        outSupplier = out;
        inSupplier = in;
    }

    public void execute() {
        if(outSupplier.get()) {
            intakeSubsystem.simplePivot();
        } else if(inSupplier.get()) {
            intakeSubsystem.revSimplePivot();
        } else {
            intakeSubsystem.pivotStop();
        }
    }
}
