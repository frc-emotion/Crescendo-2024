package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivotCommand extends Command {

    private IntakeSubsystem intakeSubsystem;

    public IntakePivotCommand(IntakeSubsystem intake){

        this.intakeSubsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggleState();
    }

    @Override
    public void execute() {
        // if(intakeSubsystem.isDown()){
        //     intakeSubsystem.setReference(IntakeConstants.INTAKE_DOWN_POSITION);
        // } else {
        //     intakeSubsystem.setReference(IntakeConstants.INTAKE_UP_POSITION);
        // }
        
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
