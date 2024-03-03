package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TimeDriveAuto extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public TimeDriveAuto(SwerveSubsystem swerve) {
        swerveSubsystem = swerve;
    }

    public void execute() {
        swerveSubsystem.driveRobotRelative(new ChassisSpeeds(1, 0, 0));
    }
}
