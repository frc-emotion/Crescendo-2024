package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class ShooterManualCommand extends Command {

    private final Supplier<Boolean> shooterSupplier;
    private final Supplier<Double> feederSupplier;
    private final ShooterSubsystem shooterSubsystem;

    private boolean feederState;
    private boolean hasIndexed;

    public ShooterManualCommand(
        Supplier<Boolean> shooterSupplier,
        Supplier<Double> feederSupplier,
        ShooterSubsystem shooterSubsystem
    ) {
        this.shooterSupplier = shooterSupplier;
        this.feederSupplier = feederSupplier;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

        feederState = false;
        hasIndexed = false;
    }

    @Override
    public void execute() {
        if (shooterSupplier.get()) {
            if(feederState) {
                shooterSubsystem.setFeederSpeed(
                    ShooterConstants.kFeedSpeed
                );
                if(hasIndexed) {
                    hasIndexed = false;
                }
            } else {
                shooterSubsystem.stopShooter();
            }
            feederState = !feederState;
        }

            // Stops the shooter once indexed
        if(shooterSubsystem.isProjectileFed() && !hasIndexed) {
            shooterSubsystem.stopFeeder();
            hasIndexed = true;
            try {
                wait(100);
            } catch(InterruptedException iex) {
                
                end(true);
            }
        }


        if (feederSupplier.get() > OIConstants.SHOOTER_DEADZONE) {
            shooterSubsystem.setShooterVelocity(
                feederSupplier.get() * ShooterConstants.kShootSpeedRotationsPerSecond
            );
        } else {
            shooterSubsystem.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFeeder();
        shooterSubsystem.stopShooter();
    }
}
