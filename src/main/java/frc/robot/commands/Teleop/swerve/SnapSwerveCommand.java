package frc.robot.commands.Teleop.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SnapSwerveCommand extends AbstractSwerveXboxCommand {

    private final PIDController thetaController = new PIDController(
        AutoConstants.kPThetaController, 
        AutoConstants.kIThetaController, 
        AutoConstants.kDThetaController
    );

    private int direction;
    
    public SnapSwerveCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> turningSpdFunc,
        int direction
     ) {
        super(swerveSubsystem, xSpdFunc, ySpdFunc, turningSpdFunc);

        thetaController.enableContinuousInput(-180, 180);
        
        this.direction = direction;
    }
    
    @Override
    public void execute() {
        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, 
            ySpeed, 
            thetaController.calculate(swerveSubsystem.getHeading_180(), direction), 
            swerveSubsystem.getRotation2d()
        );

        sendSpeedsToSubsystem();
    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }
}
