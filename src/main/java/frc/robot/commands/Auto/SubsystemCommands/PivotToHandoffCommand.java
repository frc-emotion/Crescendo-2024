package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToHandoffCommand extends Command {
    private PivotSubsystem m_PivotSubsystem;

    public PivotToHandoffCommand(PivotSubsystem subsystem) {
        this.m_PivotSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_PivotSubsystem.goToHandoff();
    }

    @Override
    public boolean isFinished() {
        return m_PivotSubsystem.isHandoffOk();
    }

    @Override
    public void end(boolean interrupted) {
        m_PivotSubsystem.stop();
    }
}
