package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutoCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public ShooterAutoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    public void execute() {
        
    }
    
}
    