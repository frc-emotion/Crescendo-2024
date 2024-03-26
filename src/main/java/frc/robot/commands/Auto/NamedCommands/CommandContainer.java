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

/**
 * Container class for most of the Commands and Command Groups used during the Autonomous period.
 */
public class CommandContainer {

    /**
     * Creates a new SequentialCommandGroup to delay for 0.25 seconds after a note is picked up.
     * @param intakeSubsystem   The Intake Subsystem
     * @return  The SequentialCommandGroup
     */
    @Deprecated
    public static SequentialCommandGroup intakeNote(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
                new IntakeDriveAutoCommand(intakeSubsystem),
                new WaitCommand(0.25));
    }

    /**
     * Decorates and returns ResetPivotAutoCommand with a timeout and a condition for whether or
     * not the pivot is already at the default position.
     * @param pivot The Pivot Subsystem
     * @return  The decorated Command
     */
    public static Command resetPivot(PivotSubsystem pivot) {
        return new ResetPivotAutoCommand(pivot).withTimeout(3.5).onlyIf(() -> !pivot.isHandoffOk());
    }

    /**
     * Runs swerve forward for a certain period of time at 2 meters per second.
     * @param swerveSubsystem   The Swerve Subsystem
     * @param seconds   The amount of time to drive for
     * @return  The timed drive Command
     */
    @Deprecated
    public static Command timeDrive(SwerveSubsystem swerveSubsystem, double seconds) {
        return new Command() {
            public void execute() {
                swerveSubsystem.driveRobotRelative(new ChassisSpeeds(2, 0, 0));
            }
        }.withTimeout(seconds);
    }

    /**
     * Retrieves the Command to toggle the intake and auto handoff.
     * @param intakeSubsystem
     * @param shooterSubsystem
     * @return
     */
    public static SequentialCommandGroup fullToggleIntake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
                new IntakePivotCommand(intakeSubsystem),
                getHandoffCommand(intakeSubsystem, shooterSubsystem));
    }

    /**
     * Retrieves the AutoHandoff command, decorated with
     * a timeout and condition.
     * @param intakeSubsystem   The Intake Subsystem object to be used
     * @param shooterSubsystem  The Shooter Subystem object to be used
     * @return  
     */
    public static Command getHandoffCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        return new HandoffAutoCommand(intakeSubsystem, shooterSubsystem, true)
                .withTimeout(2)
                .onlyIf(
                        () -> !intakeSubsystem.getBeamState() && !shooterSubsystem.isProjectileFed());
    }
}
