package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.other.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Unfinished class, will calculate the speed to shoot the note at depending on the range */
public class DynamicShootSpeaker extends ShootSpeaker {
    protected final VisionSubsystem visionSubsystem;
    public DynamicShootSpeaker(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, VisionSubsystem visionSubsystem) {
        super(shooterSubsystem, feederSubsystem);
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        shootSpeed = calculateShootSpeed();
    }

    protected double calculateShootSpeed() {
        return 0;
    }
}
