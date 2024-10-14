package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotFeed extends Command{

    private PivotSubsystem pivotSubsystem;
    public PivotFeed(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        pivotSubsystem.setRev(0);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.isAtTarget();
    }
}



    
