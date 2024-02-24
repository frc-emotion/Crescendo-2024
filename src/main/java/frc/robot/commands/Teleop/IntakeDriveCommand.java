package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDriveCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> rightBumper, leftBumper;

    public IntakeDriveCommand(IntakeSubsystem intake, 
    Supplier<Boolean> rightBumper, 
    Supplier<Boolean> leftBumper
    ){

        this.intakeSubsystem = intake;
        this.rightBumper = rightBumper;
        this.leftBumper = leftBumper;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(rightBumper.get()){
            intakeSubsystem.intakeForward();
        }
        else if(leftBumper.get()){
            intakeSubsystem.intakeReverse();
        }
        else {
            intakeSubsystem.intakeStop();
        }

        // if (leftAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
        //     intakeSubsystem.revSimplePivot();
        // } else if (rightAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
        //     intakeSubsystem.simplePivot();
        // } else {
        //     intakeSubsystem.pivotStop();
        // }
        
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
