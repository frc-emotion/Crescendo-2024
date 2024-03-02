package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command  {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        //shooterSubsystem.setTargetRPM(ShooterConstants.kShootSpeedRotationsPerSecond);
    }
    
    @Override
    public void execute() {
        shooterSubsystem.setShooterVelocity(4000);
        if(shooterSubsystem.isAtTarget(4000)) {
            shooterSubsystem.setFeederSpeed(0.15);
            intakeSubsystem.intakeForward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopFeeder();
    }
}
