package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HandoffAutoCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;

    public HandoffAutoCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
        intakeSubsystem = intake;
        feederSubsystem = feeder;
        addRequirements(intakeSubsystem, feederSubsystem);
    }

    @Override
    public void execute() {
        feederSubsystem.set(IntakeConstants.SHOOTER_TRANSFER_SPEED);
        intakeSubsystem.setIntake(IntakeConstants.SHOOTER_TRANSFER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopDrive();
        feederSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return feederSubsystem.getBeamState();
    }
}
