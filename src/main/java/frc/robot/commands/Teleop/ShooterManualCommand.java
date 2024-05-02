package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.Supplier;

public class ShooterManualCommand extends Command {

    private final Supplier<Boolean> shooterSupplier;
    private final ShooterSubsystem shooterSubsystem;

    // private boolean feederState;
    // private boolean hasIndexed;

    public ShooterManualCommand(
            Supplier<Boolean> shooterSupplier,
            ShooterSubsystem shooterSubsystem) {
        this.shooterSupplier = shooterSupplier;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

        // feederState = false;
        // hasIndexed = false;
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.updatePID();
    }

    @Override
    public void execute() {
        if (shooterSupplier.get()) {
            /*
             * if(feederState) {
             * shooterSubsystem.setFeederSpeed(
             * ShooterConstants.kFeedSpeed
             * );
             * if(hasIndexed) {
             * hasIndexed = false;
             * }
             * } else {
             * shooterSubsystem.stopShooter();
             * }
             * feederState = !feederState;
             */

            shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM); // default speed is 4000, amp is
                                                                                     // 1250
            // shooterSubsystem.setShooterRaw(0.3);

        } else {
            shooterSubsystem.setShooterRaw(0);
            // shooterSubsystem.setShooterVelocity(0);
        }

        /*
         * // Stops the shooter once indexed
         * if(shooterSubsystem.isProjectileFed() && !hasIndexed) {
         * shooterSubsystem.stopFeeder();
         * hasIndexed = true;
         * try {
         * wait(100);
         * } catch(InterruptedException iex) {
         * end(true);
         * }
         * }
         */

        // if (feederSupplier.get() > OIConstants.SHOOTER_DEADZONE) {
        // shooterSubsystem.setFeederSpeed(ShooterConstants.kShootSpeedRotationsPerSecond);
        // } else {
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}
