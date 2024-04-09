package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.SubsystemCommands.ClimbAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.HandoffAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Auto.SubsystemCommands.ResetPivotAutoCommand;
import frc.robot.commands.Teleop.IntakePivotCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.AutoManager;

/**
 * Container class for most of the Commands and Command Groups used during the
 * Autonomous period.
 */
public class CommandContainer {

    /**
     * Creates a new SequentialCommandGroup to delay for 0.25 seconds after a note
     * is picked up.
     * 
     * @param intakeSubsystem The Intake Subsystem
     * @return The SequentialCommandGroup
     */
    @Deprecated
    public static SequentialCommandGroup intakeNote(IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
                new IntakeDriveAutoCommand(intakeSubsystem),
                new WaitCommand(0.25));
    }

    /**
     * Decorates and returns ResetPivotAutoCommand with a timeout and a condition
     * for whether or
     * not the pivot is already at the default position.
     * 
     * @param pivot The Pivot Subsystem
     * @return The Command decorated as a ConditionalCommand
     */
    public static Command resetPivot(PivotSubsystem pivot) {
        return new ResetPivotAutoCommand(pivot);// .onlyIf(() -> !pivot.isHandoffOk());
    }

    /**
     * Runs swerve forward for a certain period of time at 2 meters per second.
     * 
     * @param swerveSubsystem The Swerve Subsystem
     * @param seconds         The amount of time to drive for
     * @return The timed drive Command
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
     * Retrieves the Command to toggle the intake and auto handoff. Uses the
     * getHandoffCommand() method to retrieve the handoff Command sequence.
     * 
     * @param intakeSubsystem  The Intake Subsystem object to be used in the Command
     * @param shooterSubsystem The Shooter Subsystem object to be used in the
     *                         Command
     * @return The SequentialCommandGroup which toggles the intake and runs the
     *         AutoHandoff sequence.
     */
    public static SequentialCommandGroup fullToggleIntake(IntakeSubsystem intakeSubsystem,
            FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem) {
        return new SequentialCommandGroup(
                new IntakePivotCommand(intakeSubsystem),
                getHandoffCommandGroup(intakeSubsystem, feederSubsystem, pivotSubsystem));
    }

    /**
     * Retrieves a SequentialCommandGroup containing the required sequence in order
     * to automatically
     * handoff the note to the feeder mechanism.
     * 
     * @param intakeSubsystem  The Intake Subsystem object to be used in the Command
     *                         Group
     * @param shooterSubsystem The Shooter Subystem object to be used in the Command
     *                         Group
     * @param pivotSubsystem   The Pivot Subsystem object to be used in the Command
     *                         Group
     * @return The SequentialCommandGroup with the pivot reset Command and the
     *         AutoHandoff Command
     */
    public static SequentialCommandGroup getHandoffCommandGroup(IntakeSubsystem intakeSubsystem,
            FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem) {
        return new SequentialCommandGroup(
                resetPivot(pivotSubsystem).onlyIf(() -> !pivotSubsystem.isHandoffOk()), // Check to make sure the intake
                                                                                        // shows true when retracted.
                new HandoffAutoCommand(intakeSubsystem, feederSubsystem)
                        .withTimeout(2)
                        .onlyIf(() -> !intakeSubsystem.getBeamState()));
    }

    /**
     * Retrieves a SequentialCommandGroup containing the required sequence in order
     * to automatically
     * handoff the note to the feeder mechanism. Does not have a timeout during the
     * handoff period.
     * Handoff Command will not end if the intake beam sensor does not function
     * properly.
     * 
     * @param intakeSubsystem  The Intake Subsystem object to be used in the Command
     *                         Group
     * @param shooterSubsystem The Shooter Subystem object to be used in the Command
     *                         Group
     * @param pivotSubsystem   The Pivot Subsystem object to be used in the Command
     *                         Group
     * @return The SequentialCommandGroup with the pivot reset Command and the
     *         AutoHandoff Command
     */
    public static SequentialCommandGroup getAutoHandoffCommandGroup(IntakeSubsystem intakeSubsystem,
            FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem) {
        return new SequentialCommandGroup(
                resetPivot(pivotSubsystem).onlyIf(() -> !pivotSubsystem.isHandoffOk()), // Check to make sure the intake
                                                                                        // shows true when retracted.
                new HandoffAutoCommand(intakeSubsystem, feederSubsystem)
                        .onlyIf(() -> !intakeSubsystem.getBeamState()));
    }

    /**
     * Retrieves the AutoHanoff Command decorated with a condition and timeout.
     * 
     * @param intakeSubsystem  The Intake Subsystem object to be used in the Command
     *                         Group
     * @param shooterSubsystem The Shooter Subystem object to be used in the Command
     *                         Group
     * @return A conditional ParallelRaceGroup containing the timeout and the
     *         HanoffAuto Command
     */
    public static Command getHandoffCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
        return new HandoffAutoCommand(intakeSubsystem, feederSubsystem)
                .withTimeout(2)
                .onlyIf(() -> !intakeSubsystem.getBeamState());
    }

    /**
     * Retrieves the AutoHanoff Command decorated with a condition and timeout.
     * 
     * @param intakeSubsystem  The Intake Subsystem object to be used in the Command
     *                         Group
     * @param shooterSubsystem The Shooter Subystem object to be used in the Command
     *                         Group
     * @param rev              Whether or not the Shooter should rev during the
     *                         handoff
     * @return A conditional ParallelRaceGroup containing the timeout and the
     *         HanoffAuto Command
     */
    public static Command getHandoffCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem,
            boolean rev) {
        return new HandoffAutoCommand(intakeSubsystem, feederSubsystem)
                .withTimeout(2)
                .onlyIf(() -> !intakeSubsystem.getBeamState());
    }

    /**
     * Navigates and climbs at a specific pose
     * 
     * @param climbSubsystem The Climb Subsystem
     * @param targetPose     The target pose to climb at
     * @return The Command Group to execute the climb.
     */
    public static Command getAutoClimbCommand(ClimbSubsystem climbSubsystem, Pose2d targetPose) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        AutoManager.getInstance().navigateToPose(targetPose),
                        new ClimbAutoCommand(climbSubsystem, true)),
                new ClimbAutoCommand(climbSubsystem, false));
    }
}
