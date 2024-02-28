package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.PivotAutoCommand;
import frc.robot.commands.Teleop.IntakePivotCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandContainer {
    
    public Command intakeNote(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
            new IntakePivotCommand(intakeSubsystem),
            new IntakeDriveAutoCommand(intakeSubsystem),
            new IntakePivotCommand(intakeSubsystem)
        );
    }

    public Command enRoute(PivotSubsystem pivot) {
        return new PivotAutoCommand(pivot, 1);
    }

    public Command shootSpeaker(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
            
        );
    }
}
