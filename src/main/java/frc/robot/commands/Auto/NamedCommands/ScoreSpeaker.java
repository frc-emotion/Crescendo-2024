package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Auto.SubsystemCommands.PivotAutoCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreSpeaker extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ScoreSpeaker(
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(AutoConstants.SHOOTER_SPEED_RPM);
    }
}
