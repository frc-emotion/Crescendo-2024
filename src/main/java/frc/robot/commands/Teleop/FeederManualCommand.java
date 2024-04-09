package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class FeederManualCommand extends Command {
    private FeederSubsystem feederSubsystem;
    private Supplier<Boolean> feederSupplier;

    public FeederManualCommand(FeederSubsystem feederSubsystem, Supplier<Boolean> feedSupplier) {
        this.feederSubsystem = feederSubsystem;
        this.feederSupplier = feedSupplier;
        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        if(feederSupplier.get()) {
            feederSubsystem.set(ShooterConstants.FEEDER_SPEED);
        } else {
            feederSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stop();
    }
    
}
