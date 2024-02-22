package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class ShooterManualCommand extends Command {

    private final Supplier<Boolean> shooterSupplier, feederSupplier;
    private final ShooterSubsystem shooterSubsystem;

    private boolean feederState;
    private boolean hasIndexed;

    public ShooterManualCommand(
        Supplier<Boolean> feederSupplier,
        Supplier<Boolean> shooterSupplier,
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
            /*
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
            */
            shooterSubsystem.setShooterRaw(0.3);
            
        } else {
            shooterSubsystem.stopShooter();
        }

        if(feederSupplier.get()) {
            shooterSubsystem.setFeederSpeed(-0.15);
        } else {
            shooterSubsystem.stopFeeder();
        }

        /*
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
        */


        // if (feederSupplier.get() > OIConstants.SHOOTER_DEADZONE) {
        //     shooterSubsystem.setFeederSpeed(ShooterConstants.kShootSpeedRotationsPerSecond);
        // } else {
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFeeder();
        shooterSubsystem.stopShooter();
    }
}
