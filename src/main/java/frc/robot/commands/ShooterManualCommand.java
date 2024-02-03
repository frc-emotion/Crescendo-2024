package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterManualCommand extends Command {
    private final Supplier<Boolean> shooterSupplier, feederSupplier;
    private final ShooterSubsystem shooterSubsystem;

    public ShooterManualCommand(
        Supplier<Boolean> shooterSupplier, 
        Supplier<Boolean> feederSupplier, 
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
            shooterSubsystem.setShooterSpeed(ShooterConstants.kShootSpeedRotationsPerSecond);
        } else {
            shooterSubsystem.stopShooter();
        }
        if(feederSupplier.get()) {
            shooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
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
