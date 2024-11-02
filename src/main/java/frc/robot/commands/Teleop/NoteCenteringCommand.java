package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class NoteCenteringCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    boolean lastBeamState;
    int iterations;

    final int MAX_ITERATIONS;

    public NoteCenteringCommand(IntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, IntakeConstants.NOTE_CENTERING_ITERATIONS);
    }

    public NoteCenteringCommand(IntakeSubsystem intakeSubsystem, int maxIterations) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

        lastBeamState = intakeSubsystem.getBeamState();
        iterations = 0;
        MAX_ITERATIONS = maxIterations;
    }

    @Override
    public void execute() {
        if(intakeSubsystem.getBeamState() != lastBeamState) {
            if(lastBeamState == true) {
                intakeSubsystem.setIntake(-IntakeConstants.CENTERING_SPEED);
            } else {
                intakeSubsystem.setIntake(IntakeConstants.CENTERING_SPEED);
            }
            lastBeamState = intakeSubsystem.getBeamState();
        }
    }

    /*@Override
    public boolean isFinished() {
     return iterations >= MAX_ITERATIONS;
    }*/

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeStop();
    }
    
}
