package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class PivotAutoCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private double targetAngle;

    public PivotAutoCommand(PivotSubsystem pivotSubsystem, double targetAngle) {
        this.pivotSubsystem = pivotSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        pivotSubsystem.setDegrees(targetAngle);

    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.isAtTarget();
    }
}
