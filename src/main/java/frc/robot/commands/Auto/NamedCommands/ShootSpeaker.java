package frc.robot.commands.Auto.NamedCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
    protected final ShooterSubsystem shooterSubsystem;

    protected double shootSpeed;
    protected Supplier<Boolean> shouldShoot;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem) {
        this(shooterSubsystem, ShooterConstants.SHOOTER_SPEED_RPM);
    }

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, double shootSpeed) {
        this(shooterSubsystem, shootSpeed, () -> true);
    }

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, double shootSpeed, Supplier<Boolean> shouldShoot) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootSpeed = shootSpeed;
        this.shouldShoot = shouldShoot;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // shootSpeed = ShooterConstants.SHOOTER_SPEED_RPM;
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(shootSpeed);
        if (shooterSubsystem.getShooterVelocity() > shootSpeed - 400 && shouldShoot.get()) {
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
