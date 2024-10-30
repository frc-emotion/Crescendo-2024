package frc.robot.commands.Auto.NamedCommands;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DynamicShootSpeaker extends ShootSpeaker {
    protected final VisionSubsystem visionSubsystem;
    // protected final Interpolator<Double> interpolartor;

    public DynamicShootSpeaker(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
        super(shooterSubsystem);
        this.visionSubsystem = visionSubsystem;
        // interpolartor = Interpolator.forDouble();
        
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        shootSpeed = calculateShootSpeed();
    }

    protected double calculateShootSpeed() {
        return ShooterConstants.MAX_SHOOT_SPEED;
    }
}
