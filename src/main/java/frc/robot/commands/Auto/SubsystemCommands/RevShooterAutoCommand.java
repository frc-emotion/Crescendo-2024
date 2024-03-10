package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterAutoCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public RevShooterAutoCommand(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(ShooterConstants.IDLE_SPEED);
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
