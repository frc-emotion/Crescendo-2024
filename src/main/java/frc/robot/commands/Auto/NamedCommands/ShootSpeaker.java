package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootSpeaker extends Command {
    protected final ShooterSubsystem shooterSubsystem;

    protected double shootSpeed;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // shootSpeed = ShooterConstants.SHOOTER_SPEED_RPM;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM);
        if (shooterSubsystem.getShooterVelocity() > ShooterConstants.SHOOTER_SPEED_RPM - 400) {
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
