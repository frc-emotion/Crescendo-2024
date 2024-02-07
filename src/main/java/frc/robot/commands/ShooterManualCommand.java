package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterManualCommand extends Command {
    private final Supplier<Boolean> shooterSupplier;
    private final Supplier<Double> feederSupplier;
    private final ShooterSubsystem shooterSubsystem;

    public ShooterManualCommand(
        Supplier<Boolean> shooterSupplier, 
        Supplier<Double> feederSupplier, 
        ShooterSubsystem shooterSubsystem
    ) {
        this.shooterSupplier = shooterSupplier;
        this.feederSupplier = feederSupplier;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        if(shooterSupplier.get()) {
            shooterSubsystem.setShooterVelocity(ShooterConstants.kShootSpeedRotationsPerSecond);
        } else {
            shooterSubsystem.stopShooter();
        }
        if(feederSupplier.get() > 0.2) {
            shooterSubsystem.setFeederSpeed(feederSupplier.get() * ShooterConstants.kFeedSpeed);
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
