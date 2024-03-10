package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.PivotAutoCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandContainer {

    public static Command intakeNote(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
                new IntakeDriveAutoCommand(intakeSubsystem),
                new WaitCommand(1));
    }

    public static Command enRoute(PivotSubsystem pivot) {
        return new PivotAutoCommand(pivot, 1);
    }

    public static Command timeDrive(SwerveSubsystem swerveSubsystem, double seconds) {
        return new Command() {
            public void execute() {
                swerveSubsystem.driveRobotRelative(new ChassisSpeeds(2, 0, 0));
            }
        }.withTimeout(seconds);
    }
}
