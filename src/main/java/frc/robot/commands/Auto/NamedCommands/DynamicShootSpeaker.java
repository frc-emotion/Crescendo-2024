package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Unfinished class, will calculate the speed to shoot the note at depending on the range */
public class DynamicShootSpeaker extends ShootSpeaker {
    protected final VisionSubsystem visionSubsystem;
    protected boolean dynamicSpeedEnabled;

        // Distance to rpm shooter map
    private final static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

        // Static initializer
    static {
        shooterMap.clear();
        shooterMap.put(1.0, ShooterConstants.SHOOTER_SPEED_RPM);
        shooterMap.put(3.07, 4500.0);
    }

    public DynamicShootSpeaker(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, VisionSubsystem visionSubsystem) {
        super(shooterSubsystem, feederSubsystem);
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        shootSpeed = calculateSpeed();
    }


    protected double calculateSpeed() {
        double distance = visionSubsystem.getDistanceTo(
                                    (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                                            ? VisionConstants.RED_SPEAKER_CENTER
                                            : VisionConstants.BLUE_SPEAKER_CENTER);
        return shooterMap.get(distance);
        
    }
}
