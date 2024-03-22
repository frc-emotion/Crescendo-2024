package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
    private ShooterSubsystem shooterSubsystem;

    private int lastShooterSpeed;

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
        shooterSubsystem.setShooterVelocity(3000);
        if (shooterSubsystem.getShooterVelocity() > ShooterConstants.MIN_SHOOT_SPEED) {
            shooterSubsystem.setFeederSpeed(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        }
        lastShooterSpeed = (int) shooterSubsystem.getShooterVelocity();
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
