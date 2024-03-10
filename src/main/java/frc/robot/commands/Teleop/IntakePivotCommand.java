package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivotCommand extends Command {

    private IntakeSubsystem intakeSubsystem;

    public IntakePivotCommand(IntakeSubsystem intake) {

        this.intakeSubsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.isUp()) {
            intakeSubsystem.setGoal(-0.32);
        } else {
            intakeSubsystem.setGoal(0);
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
