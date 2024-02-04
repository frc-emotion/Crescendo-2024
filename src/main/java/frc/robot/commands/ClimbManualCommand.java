package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Climb Manual Command
 * 
 * @author Jason Ballinger
 * @version 2/3/2024
 */
public class ClimbManualCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private final double position;

    /**
     * Create a new instance of ClimbManualCommand
     * 
     * @param cs       Climb System being affected by the command
     * @param position Position to set Climb to
     */
    public ClimbManualCommand(ClimbSubsystem cs, double position) {
        this.climbSubsystem = cs;
        this.position = position;

        addRequirements(cs);
    }

    @Override
    public void execute() {
        climbSubsystem.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }
}
