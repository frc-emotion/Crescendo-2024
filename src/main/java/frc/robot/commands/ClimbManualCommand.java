package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Climb Manual Command
 * 
 * @author Jason Ballinger
 * @version 2/3/2024
 */
public class ClimbManualCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private Supplier<Double> input;
    private double coefficient = 10;

    /**
     * Create a new instance of ClimbManualCommand
     * 
     * @param cs    Climb System being affected by the command
     * @param input Controller joystick input value
     */
    public ClimbManualCommand(ClimbSubsystem cs, Supplier<Double> input) {
        this.climbSubsystem = cs;
        this.input = input;

        addRequirements(cs);
    }

    @Override
    public void execute() {
        if (input.get() > ClimbConstants.CLIMB_DEADZONE) {
            // move up
            climbSubsystem.setPosition(climbSubsystem.getPosition() + (coefficient * input.get()));
        } else if (input.get() < -ClimbConstants.CLIMB_DEADZONE) {
            // move down
            climbSubsystem.setPosition(climbSubsystem.getPosition() - (coefficient * input.get()));
        }
        // climbSubsystem.setPosition(input.get());
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }
}
