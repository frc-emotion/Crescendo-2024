package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotIntake extends Command {

    private IntakeSubsystem intakeSubsystem;

    public PivotIntake(IntakeSubsystem intake){
        this.intakeSubsystem = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggleEndState();
    }

    @Override
    public void execute() {
        intakeSubsystem.pivot();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.pivotStop();
    }

    @Override
    public boolean isFinished(){
        return intakeSubsystem.checkCurrentSpike();
    }
}