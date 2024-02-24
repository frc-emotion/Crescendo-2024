package frc.robot.commands.Teleop.swerve.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.commands.Teleop.swerve.field.DefaultSwerveXboxCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotCentricSwerveCommand extends DefaultSwerveXboxCommand {
    
    public RobotCentricSwerveCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc
    ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc);
    }

    @Override
    public void execute() {
        super.execute();
        
        robotSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        sendSpeedsToSubsystem();
    }
}
