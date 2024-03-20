package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.SubsystemCommands.HandoffAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.PivotAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.ResetPivotAutoCommand;
import frc.robot.commands.Teleop.IntakePivotCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandContainer {

    public static Command intakeNote(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
                new IntakeDriveAutoCommand(intakeSubsystem),
                new WaitCommand(0.25));
    }

    public static Command resetPivot(PivotSubsystem pivot) {
        return new ResetPivotAutoCommand(pivot).withTimeout(3.5).onlyIf(() -> !pivot.isHandoffOk());
    }

    public static Command timeDrive(SwerveSubsystem swerveSubsystem, double seconds) {
        return new Command() {
            public void execute() {
                swerveSubsystem.driveRobotRelative(new ChassisSpeeds(2, 0, 0));
            }
        }.withTimeout(seconds);
    }

    public static Command fullToggleIntake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
            new IntakePivotCommand(intakeSubsystem),
            new HandoffAutoCommand(intakeSubsystem, shooterSubsystem).withTimeout(2).onlyIf(() -> !intakeSubsystem.getBeamState())
        );
    }

    public static Command getHandoffCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        return new HandoffAutoCommand(intakeSubsystem, shooterSubsystem)
            .withTimeout(2)
            .onlyIf(
                () -> !intakeSubsystem.getBeamState() && !shooterSubsystem.isProjectileFed()
            );
    }
}
