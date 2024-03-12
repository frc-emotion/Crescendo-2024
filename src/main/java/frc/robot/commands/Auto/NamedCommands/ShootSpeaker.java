package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning("Auto Shoot started", false);
        // shooterSubsystem.setTargetRPM(ShooterConstants.kShootSpeedRotationsPerSecond);
    }

    @Override
    public void execute() {
        DriverStation.reportWarning("Auto Shooting", false);
        shooterSubsystem.setShooterVelocity(4000);
        if (shooterSubsystem.getShooterVelocity() > 3750) {
            shooterSubsystem.setFeederSpeed(0.2);
            intakeSubsystem.setIntake(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        DriverStation.reportWarning("Auto Shoot Ended", false);
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
