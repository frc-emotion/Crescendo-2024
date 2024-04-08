package frc.robot.commands.debug;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Command to reset the gyro. It is not generally recommended to run this
 * Command
 * except when disabled because it disables the ability to drive until the
 * Command is complete.
 */
public class ResetGyroCommand extends Command {
    private SwerveSubsystem swerve;

    public ResetGyroCommand(SwerveSubsystem swerveSubsystem) {
        swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning("Gyro Calibrating", false);
        swerve.zeroHeading();
    }

    @Override
    public void end(boolean interrupted) {
        DriverStation.reportWarning("Gyro Calibration Finished", null);
    }

    @Override
    public boolean isFinished() {
        return swerve.isGyroCalibrating();
    }

    /**
     * Decorates the Command so it can run while disabled.
     * 
     * @return A WrappedCommand containing the ResetGyroCommand.
     */
    public static Command getCommand() {
        return new ResetGyroCommand(RobotContainer.m_SwerveSubsystem).ignoringDisable(true);
    }
}
