package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AutoManager;

public class Rotate extends MonitorVision {
    private AutoManager autoManager;

    public Rotate(VisionSubsystem visionSubsystem, AutoManager autoManager) {
        super(visionSubsystem);
        this.autoManager = autoManager;

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();

        Translation2d currentPose2d = visionSubsystem.getCurrentPose().getTranslation();
        // System.out.println(currentPose2d);
        Rotation2d rotation2d = getTargetRotation();
        // System.err.println(rotation2d);

        autoManager.navigateToPose(new Pose2d(currentPose2d, rotation2d));
    }

    @Override
    public boolean isFinished() {
        // return pivotSubsystem.isAtTarget(calculateAngle());
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    /**
     * 
     * @return the target angle for robot
     */
    private Rotation2d getTargetRotation() {
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

        System.out.println("Final Theta: " + (currentTheta + offsetTheta));
        return new Rotation2d(currentTheta + offsetTheta);
    }

}
