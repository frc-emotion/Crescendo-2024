package frc.robot.commands.Auto.NamedCommands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
