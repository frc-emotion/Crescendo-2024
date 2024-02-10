package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class PivotIntake extends Command {

    private IntakeSubsystem intake;

    public PivotIntake(IntakeSubsystem intake){
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
        intake.pivotStop();
    }

    @Override
    public boolean isFinished(){
        return intake.checkCurrentSpike();
    }
}