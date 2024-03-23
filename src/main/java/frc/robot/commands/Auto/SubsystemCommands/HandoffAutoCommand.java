package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HandoffAutoCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private boolean rev;

    public HandoffAutoCommand(IntakeSubsystem intake, ShooterSubsystem shooter, boolean rev) {
        intakeSubsystem = intake;
        shooterSubsystem = shooter;
        this.rev = rev;
        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        // m_ShooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
        shooterSubsystem.setFeederSpeed(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        if (rev) {
            intakeSubsystem.setIntake(IntakeConstants.AUTO_SHOOTER_TRANSFER_SPEED);
            shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM);
        } else {
            intakeSubsystem.setIntake(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeStop();
        shooterSubsystem.stopFeeder();
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.isProjectileFed();
    }
}
