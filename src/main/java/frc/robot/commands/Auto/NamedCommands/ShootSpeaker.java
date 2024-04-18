package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Waits for the Shooter to get up to speed, then shoots the note until the beam break is no longer broken. */
public class ShootSpeaker extends Command {
    protected final ShooterSubsystem shooterSubsystem;
    protected final FeederSubsystem feederSubsystem;

    protected double shootSpeed;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        addRequirements(shooterSubsystem, feederSubsystem);
    }

    @Override
    public void initialize() {
        // shootSpeed = ShooterConstants.SHOOTER_SPEED_RPM;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM);
        if (shooterSubsystem.getShooterVelocity() > ShooterConstants.SHOOTER_SPEED_RPM - 400) {
            feederSubsystem.set(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        feederSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return feederSubsystem.getBeamState();
    }
}
