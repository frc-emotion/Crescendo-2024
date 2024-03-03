package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDriveCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> forwardFunc, reverseFunc;

    public IntakeDriveCommand(IntakeSubsystem intake, 
    Supplier<Boolean> forwardFunc, 
    Supplier<Boolean> reverseFunc
    ){

        this.intakeSubsystem = intake;
        this.forwardFunc = forwardFunc;
        this.reverseFunc = reverseFunc;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /*
        if (leftAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
            intakeSubsystem.revSimplePivot();
        } else if (rightAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
            intakeSubsystem.simplePivot();
        } else {
            intakeSubsystem.pivotStop();
        }
        */

        if(forwardFunc.get() && intakeSubsystem.getBeamState()){
            intakeSubsystem.intakeForward();
        } else if(reverseFunc.get()){
            intakeSubsystem.intakeReverse();
        } else {
            intakeSubsystem.intakeStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
