package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterFeederAutoCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public ShooterFeederAutoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
    }

    @Override
    public void execute() {
        if (shooterSubsystem.isProjectileFed()) {
            shooterSubsystem.stopFeeder();
        }
    }

}
