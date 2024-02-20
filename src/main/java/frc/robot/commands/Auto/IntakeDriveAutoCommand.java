package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDriveAutoCommand extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;
    private PersianState persianState;

    public enum PersianState {
        ON,
        REVERSE,
        OFF;
    }
    
    public IntakeDriveAutoCommand(IntakeSubsystem intakeSubsystem, PersianState persianState) {
        this.intakeSubsystem = intakeSubsystem;
        this.persianState = persianState;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        switch (persianState) {
            case ON:
                intakeSubsystem.intakeForward();
            case REVERSE:
                intakeSubsystem.intakeReverse();
            case OFF:
                intakeSubsystem.intakeStop();  
        }
    }
}
