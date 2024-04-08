package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.other.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class DynamicShootSpeaker extends ShootSpeaker {
    protected final VisionSubsystem visionSubsystem;
    public DynamicShootSpeaker(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
        super(shooterSubsystem);
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
