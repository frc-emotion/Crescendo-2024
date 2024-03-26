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
        if (intakeSubsystem.isUp()) {
            intakeSubsystem.setGoal(IntakeConstants.DEPLOYED_POS);
        } else {
            intakeSubsystem.setGoal(IntakeConstants.RETRACTED_POS);
        }
        // if(shouldBeDown) {
        // intakeSubsystem.setGoal(-0.32);
        // }
        // else {
        // intakeSubsystem.setGoal(0);
        // }
    }

    @Override
    public void execute() {
        // if (leftAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
        // intakeSubsystem.revSimplePivot();
        // } else if (rightAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
        // intakeSubsystem.simplePivot();
        // }
        // if(intakeSubsystem.isDown()){
        // intakeSubsystem.revSimplePivot();
        // } else {
        // intakeSubsystem.simplePivot();
        // }

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
