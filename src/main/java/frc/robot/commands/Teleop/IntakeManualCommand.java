package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> rightBumper, leftBumper;

    public IntakeManualCommand(IntakeSubsystem intake, 
    Supplier<Boolean> rightBumper, 
    Supplier<Boolean> leftBumper){

        this.intakeSubsystem = intake;
        this.rightBumper = rightBumper;
        this.leftBumper = leftBumper;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggleEndState();
    }

    @Override
    public void execute() {
        if(rightBumper.get()){
            intakeSubsystem.intakeForward();
        }
        else {
            intakeSubsystem.intakeStop();
        }

        if(leftBumper.get()){
            intakeSubsystem.intakeReverse();
        }
        else {
            intakeSubsystem.intakeStop();
        }
        
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
