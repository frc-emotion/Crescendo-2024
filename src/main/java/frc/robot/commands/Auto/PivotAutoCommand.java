package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotAutoCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private int targetPreset;
    
    public PivotAutoCommand(PivotSubsystem pivotSubsystem, int targetPreset) {
        this.pivotSubsystem = pivotSubsystem;
        this.targetPreset = targetPreset;
        addRequirements(pivotSubsystem);
    }
    
    @Override
    public void initialize() {
        pivotSubsystem.setIndex(targetPreset);
    }

    @Override
    public void execute() {
        pivotSubsystem.goToPreset();
    
    
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false; // pivotSubsystem.isAtTarget();
    }
}
