package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutoCommand extends Command  {

    private ShooterSubsystem shooterSubsystem;
    private boolean hasFinished;

    public ShooterAutoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        hasFinished = false;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
        shooterSubsystem.setShooterVelocity(ShooterConstants.kShootSpeedRotationsPerSecond);
    }
    
    @Override
    public void execute() {
        double velocity = shooterSubsystem.getShooterVelocity();

        if(shooterSubsystem.isProjectileFed()) {    // Stops feeder when projectile is in proper position
            shooterSubsystem.stopFeeder();
        } else if (velocity > ShooterConstants.kMaxOutput - ShooterConstants.kMaxOutputError) {     // Shoots when the shooter has reached full speed, accounting for error
            shooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
            try {
                wait(500);
            } catch(InterruptedException iex) {
                hasFinished = true;
                end(true);
                cancel();
                return;
            } finally {
                hasFinished = true;
                end(false);
                cancel();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return hasFinished;
    }
}
