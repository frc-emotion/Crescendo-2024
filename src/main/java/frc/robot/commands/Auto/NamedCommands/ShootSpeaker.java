package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
    protected final ShooterSubsystem shooterSubsystem;

    protected double shootSpeed;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        shootSpeed = ShooterConstants.SHOOTER_SPEED_RPM;
    }

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, double shootSpeed) {
        this(shooterSubsystem);
        this.shootSpeed = shootSpeed;
    }

    @Override
    public void initialize() {
        // shootSpeed = ShooterConstants.SHOOTER_SPEED_RPM;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(shootSpeed);
        if (shooterSubsystem.getShooterVelocity() > shootSpeed - 400) {
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
