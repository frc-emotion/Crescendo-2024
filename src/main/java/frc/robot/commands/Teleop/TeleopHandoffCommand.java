package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopHandoffCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Supplier<Boolean> shooterRevSupplier;

    public TeleopHandoffCommand(Supplier<Boolean> shooterRevSupplier, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterRevSupplier = shooterRevSupplier;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setFeederSpeed(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        intakeSubsystem.setIntake(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        
        if (shooterRevSupplier.get()) {;
            shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM);
        }
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.isProjectileFed();
    }
}
