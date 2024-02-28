package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDriveCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> rightBumper, leftBumper;
    private final Supplier<Double> leftAxis, rightAxis;

    public IntakeDriveCommand(IntakeSubsystem intake, 
    Supplier<Boolean> rightBumper, 
    Supplier<Boolean> leftBumper,
    Supplier<Double> leftAxis,
    Supplier<Double> rightAxis
    ){

        this.intakeSubsystem = intake;
        this.rightBumper = rightBumper;
        this.leftBumper = leftBumper;
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (leftAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
            intakeSubsystem.revSimplePivot();
        } else if (rightAxis.get() > Constants.OIConstants.INTAKE_DEADZONE) {
            intakeSubsystem.simplePivot();
        } else {
            intakeSubsystem.pivotStop();
        }

        //if beambreak not triggered
        // continue with triggers
        if(!intakeSubsystem.getBeamState()) {
            intakeSubsystem.intakeStop();
            return;
        }

        if(rightBumper.get()){
            intakeSubsystem.intakeForward();
        }
        else if(leftBumper.get()){
            intakeSubsystem.intakeReverse();
        }
        else {
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
