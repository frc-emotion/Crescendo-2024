package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurboModeSwerveCommand  extends AbstractSwerveXboxCommand {
    public TurboModeSwerveCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc,
        Supplier<Boolean> resetHeadingFunc,
        Supplier<Boolean> fieldOrientedFunc
        //Supplier<Boolean> slowModeFunc,
        //Supplier<Boolean> turboModeFunc,
        //Supplier<Double> hardLeft,
        //Supplier<Double> hardRight
    ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc, resetHeadingFunc, fieldOrientedFunc);
    }

    @Override
    public void execute() {
        super.execute();

        xSpeed *= 2;
        ySpeed *= 2;
        turningSpeed *= 2;

        sendSpeedsToSubsystem();
    }
}
