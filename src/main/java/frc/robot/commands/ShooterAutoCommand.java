package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutoCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public ShooterAutoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    public void execute() {
        double velocity = shooterSubsystem.getShooterVelocity();
        boolean hasShot = false;

        shooterSubsystem.setShooterVelocity(ShooterConstants.kShootSpeedRotationsPerSecond);
        while(true) {
            if(velocity > ShooterConstants.kMaxOutput - ShooterConstants.kMaxOutputError) {
                shooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
                hasShot = true;
            } else if(hasShot) {
                break;
            }
        }   
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopFeeder();
    }
}
    