package frc.robot.commands.Teleop.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Theoretically rotates the robot to face a certain direction while not moving
 */
public class SnapCommand extends Command {

    private SwerveSubsystem swerveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ChassisSpeeds robotSpeeds;

    public SnapCommand(
            SwerveSubsystem swerveSubsystem,
            VisionSubsystem visionSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.updatePID();
    }

    @Override
    public void execute() {
        super.execute();

        //Translation2d currentPose2d = visionSubsystem.getCurrentPose().getTranslation();
        // System.out.println(currentPose2d);
        double theta = getTargetRotation();

        double velocity = swerveSubsystem.calculateThetaPID(swerveSubsystem.getHeading(), theta, false);

        System.out.println("Velocity: " + velocity);

        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                velocity,
                swerveSubsystem.getRotation2d());

        swerveSubsystem.driveRobotRelative(robotSpeeds);
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.thetaPIDAtSetpoint(false);
    }

        /**
     * 
     * @return the target angle for robot
     */
    private double getTargetRotation() {
        double yOffset;

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            yOffset = VisionConstants.RED_SPEAKER_CENTER.getY() - visionSubsystem.getCurrentPose().getY();
        } else {
            yOffset = VisionConstants.BLUE_SPEAKER_CENTER.getY() - visionSubsystem.getCurrentPose().getY();
        }

        System.out.println("Y Offset" + yOffset);
        double currentTheta = visionSubsystem.getTheta();
        System.out.println("Current Theta: " + currentTheta);
        double distance = visionSubsystem.getDistanceTo(
                (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? VisionConstants.RED_SPEAKER_CENTER
                        : VisionConstants.BLUE_SPEAKER_CENTER);
        System.out.println("Distance: " + distance);
        double offsetTheta = Math.acos(yOffset / distance);
        System.out.println("Offset Theta: " + offsetTheta);

        System.out.println("Final Theta: " + Units.radiansToDegrees(currentTheta + offsetTheta));
        return Units.radiansToDegrees(currentTheta + offsetTheta);
    }
}
