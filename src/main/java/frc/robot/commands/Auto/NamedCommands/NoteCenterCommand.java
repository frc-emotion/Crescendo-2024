package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class NoteCenterCommand extends Command {
    private IntakeSubsystem intakeSubsystem;

    private final int targetIterations;

    private int iterations;
    private double firstPos;
    private boolean noteBackedUp;

    public NoteCenterCommand(IntakeSubsystem intakeSubsystem, int iterations) {
        this.intakeSubsystem = intakeSubsystem;
        targetIterations = iterations;
    }

    public void initialize() {
        firstPos = intakeSubsystem.getDrivePos();
        noteBackedUp = false;
        iterations = 0;
    }

    public void execute() {
        if(!noteBackedUp) {
            intakeSubsystem.setIntake(-IntakeConstants.kNoteCenterSpeed);
            if(intakeSubsystem.getDrivePos() <= (firstPos - IntakeConstants.NOTE_CENTER_DISTANCE)) {
                noteBackedUp = true;
            }
        } else if(noteBackedUp) {
            intakeSubsystem.setIntake(IntakeConstants.kNoteCenterSpeed);
            if(intakeSubsystem.getDrivePos() >= firstPos) {
                noteBackedUp = false;
                iterations++;
            }
        }
    }

    // public void execute() {
    //     if(intakeSubsystem.getBeamState()) {
    //         intakeSubsystem.setIntake(-IntakeConstants.kNoteCenterSpeed);
    //     } else {
    //         intakeSubsystem.setIntake(IntakeConstants.kNoteCenterSpeed);
    //     }
    // }

    public boolean isFinished() {
        return iterations >= targetIterations;
    }

    public void end() {
        intakeSubsystem.stopDrive();
    }
    
}
