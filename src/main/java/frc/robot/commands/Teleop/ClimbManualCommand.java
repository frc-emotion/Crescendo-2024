package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.Supplier;

/**
 * Climb Xbox Command
 *
 * @author Jason Ballinger
 * @version 2/3/2024
 */
public class ClimbManualCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private Supplier<Double> input;
    private double coefficient = 10;

    /**
     * Create a new instance of ClimbXboxCommand
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
        /*
        if (input.get() > OIConstants.CLIMB_DEADZONE) {
            // // move up
            // climbSubsystem.setPosition(
            //     climbSubsystem.getPosition() + (coefficient * input.get())
            // );
            climbSubsystem.rawClimbUp();
        } else if (input.get() < -OIConstants.CLIMB_DEADZONE) {
            // move down
            climbSubsystem.setPosition(
                climbSubsystem.getPosition() - (coefficient * input.get())
            );
        }*/
        if (Math.abs(input.get()) > ClimbConstants.DEADZONE) {
            climbSubsystem.setRawSpeed(input.get());
        } else {
            climbSubsystem.stop();

        }
        
        // climbSubsystem.setPosition(input.get());
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }
}
