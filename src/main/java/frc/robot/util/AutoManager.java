package frc.robot.util;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoManager {

    private VisionSubsystem visionSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private static final PathConstraints kPathConstraints = new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);

    public AutoManager(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        AutoBuilder.configureHolonomic(
                this.visionSubsystem::getCurrentPose, // If this has issues switch to odometry only based pose
                this.visionSubsystem::resetOdometry,
                this.swerveSubsystem::getChassisSpeeds,
                this.swerveSubsystem::driveRobotRelative,
                
                new HolonomicPathFollowerConfig(
                        new PIDConstants(AutoConstants.kPXController),
                        new PIDConstants(AutoConstants.kPThetaController),
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kWheelBase,
                        new ReplanningConfig()),

                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                swerveSubsystem);

        Pathfinding.setPathfinder(new LocalADStar());
    }

    
    public Command getAutoCommand(String name) {
        return AutoBuilder.buildAuto(name);
    }

    /**
     * Creates a new Command to pathfind to a certain pose using
     * only the target pose.
     * 
     * @param pose The target pose
     * @return The new command to reach the target pose
     */
    public Command navigateToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
                pose,
                kPathConstraints);
    }

    /**
     * Creates a new Command to pathfind to a certain pose using
     * the target pose and end velocity.
     * 
     * @param pose        The target pose
     * @param endVelocity The end velocity
     * @return The new command to reach the target pose
     */
    public Command navigateToPose(Pose2d pose, double endVelocity) {
        return AutoBuilder.pathfindToPose(
                pose,
                kPathConstraints,
                endVelocity);
    }

    /**
     * Creates a new Command to pathfind to a certain pose using
     * the target pose, end velocity, and the distance the robot
     * should travel before turning to the target pose's heading.
     * 
     * @param pose                  The target pose
     * @param endVelocity           The end velocity
     * @param rotationDelayDistance The distance the robot should travel before
     *                              turning
     * @return The new Command to reach the target pose
     */
    public Command navigateToPose(Pose2d pose, double endVelocity, double rotationDelayDistance) {
        return AutoBuilder.pathfindToPose(
                pose,
                kPathConstraints,
                endVelocity,
                rotationDelayDistance);
    }
}
