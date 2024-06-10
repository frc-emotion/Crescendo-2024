package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

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
        intakeSubsystem.updatePID();
        if (intakeSubsystem.isUp()) {
            intakeSubsystem.setGoal(IntakeConstants.DEPLOYED_POS, IntakeConstants.DEPLOYED_POS);
        } else {
            
            intakeSubsystem.setGoal(IntakeConstants.RETRACTED_POS, IntakeConstants.RETRACTED_POS);
        }
    }

    @Override
    public void execute() {
        intakeSubsystem.travelToSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.toggleState();
        intakeSubsystem.pivotStop();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.hasReachedSetpoint();
    }
}
