package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Toggles the current state of the intake (deployed or retracted).
 */
public class IntakePivotCommand extends Command {

    private IntakeSubsystem intakeSubsystem;

    public IntakePivotCommand(IntakeSubsystem intake) {

        this.intakeSubsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (!intakeSubsystem.isDown()) {
            intakeSubsystem.setPivotTarget(IntakeConstants.DEPLOYED_POS);
        } else {
            intakeSubsystem.setPivotTarget(IntakeConstants.RETRACTED_POS);
        }
    }

    @Override
    public void execute() {
        intakeSubsystem.goToPivotSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.togglePivot();
        intakeSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isPivotAtTarget();
    }
}
