package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
    private ShooterSubsystem shooterSubsystem;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // shooterSubsystem.setTargetRPM(ShooterConstants.kShootSpeedRotationsPerSecond);
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM);
        if (shooterSubsystem.getShooterVelocity() > ShooterConstants.MIN_SHOOT_SPEED) {
            shooterSubsystem.setFeederSpeed(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return !shooterSubsystem.isProjectileFed();
    }
}
