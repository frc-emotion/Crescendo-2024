package frc.robot.commands.vision;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Auto.SubsystemCommands.IntakeDriveAutoCommand;
import frc.robot.commands.Teleop.IntakePivotCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class VisionCommandFactory {

    /** Returns a Command which picks up a note detected by the VisionSubsystem */
    public static Command getNotePickupCommand(VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem) {
        Optional<Pose2d> notePose = visionSubsystem.getObjectPose();
        if(notePose.isPresent()) {
            return new ParallelCommandGroup(
                AutoBuilder.pathfindToPose(notePose.get(), AutoConstants.NAVIGATE_CONSTRAINTS, AutoConstants.AUTO_NOTE_PICKUP_END_VELOCITY),
                new SequentialCommandGroup(
                    new IntakePivotCommand(intakeSubsystem),
                    new IntakeDriveAutoCommand(intakeSubsystem)
                )
            );
        } else {
            return new InstantCommand();
        }
    }
}
