package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class ResetPivotAutoCommand extends Command {
    private PivotSubsystem pivotSubsystem;

    public ResetPivotAutoCommand(PivotSubsystem pivot) {
        pivotSubsystem = pivot;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        pivotSubsystem.goToHandoff();
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.isAtTarget(60);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stop();
    }
}
