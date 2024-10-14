package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterCustomCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double shooterGoal;

    public RevShooterCustomCommand(ShooterSubsystem shooter, double goal) {
        shooterSubsystem = shooter;
        shooterGoal=goal;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(shooterGoal);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
