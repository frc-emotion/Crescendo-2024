package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.Auto.SubsystemCommands.PivotAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.ShooterAutoCommand;


public class ScoreSpeaker{
    private final ShooterSubsystem shooterSubsystem;
    private final PivotSubsystem pivotSubsystem;
    
    public ScoreSpeaker(
        ShooterSubsystem shooterSubsystem, 
        PivotSubsystem pivotSubsystem
        ) {
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;

    }

    public Command daCommand() {

        return new SequentialCommandGroup(
            new PivotAutoCommand(pivotSubsystem, 0).withTimeout(0), // TODO: Set preset
            new ShooterAutoCommand(shooterSubsystem).withTimeout(0)
            );
    }
}