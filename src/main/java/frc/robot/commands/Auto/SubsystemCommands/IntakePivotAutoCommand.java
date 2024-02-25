package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivotAutoCommand extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;

    public IntakePivotAutoCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggleState();

    }

    @Override
    public void execute() {
        // if(intakeSubsystem.isDown()){
        //     intakeSubsystem.setReference(IntakeConstants.INTAKE_DOWN_POSITION);
        // } else {
        //     intakeSubsystem.setReference(IntakeConstants.INTAKE_UP_POSITION);
        // }
    }

    public static Command getAutoCommand() {
        return new IntakePivotAutoCommand(RobotContainer.m_IntakeSubsystem).withTimeout(AutoConstants.INTAKE_TIMEOUT);
    }
}
