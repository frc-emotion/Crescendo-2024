package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDriveAutoCommand extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;

    public IntakeDriveAutoCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeForward();
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getBeamState();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeStop();
    }
}
