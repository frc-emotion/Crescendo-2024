package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class PivotIntake extends Command {

    private Intake intake;

    public PivotIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.toggleEndState();
    }

    @Override
    public void execute() {
        intake.pivot();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

