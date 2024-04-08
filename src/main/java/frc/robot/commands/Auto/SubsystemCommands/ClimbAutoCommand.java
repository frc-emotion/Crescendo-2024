package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbAutoCommand extends Command {
    private ClimbSubsystem climbSubsystem;
    private boolean shouldExtend;

    public ClimbAutoCommand(ClimbSubsystem climb, boolean extend) {
        climbSubsystem = climb;
        shouldExtend = extend;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        if (shouldExtend)
            climbSubsystem.setRawSpeed(ClimbConstants.kSpeed);
        else
            climbSubsystem.setRawSpeed(-ClimbConstants.kSpeed);
    }

    @Override
    public boolean isFinished() {
        return (climbSubsystem.getPosition() >= ClimbConstants.EXTENSION_LIMIT && shouldExtend)
                || (climbSubsystem.getPosition() <= 0 && !shouldExtend);
    }
}
