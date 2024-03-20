package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HandoffAutoCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public HandoffAutoCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
        intakeSubsystem = intake;
        shooterSubsystem = shooter;
        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntake(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        // m_ShooterSubsystem.setFeederSpeed(ShooterConstants.kFeedSpeed);
        shooterSubsystem.setFeederSpeed(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM);
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
