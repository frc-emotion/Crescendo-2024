package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;
import java.util.function.Supplier;

public class LEDTeleopCommand extends Command {

    private final LEDSubsystem ls;
    private final Supplier<Boolean> intakeBreakSupplier;
    private final Supplier<Boolean> shooterBreakSupplier;

    public LEDTeleopCommand(
        LEDSubsystem ls,
        Supplier<Boolean> intakeBreakSupplier,
        Supplier<Boolean> shooterBreakSupplier
    ) {
        this.ls = ls;
        this.intakeBreakSupplier = intakeBreakSupplier;
        this.shooterBreakSupplier = shooterBreakSupplier;

        addRequirements(ls);
    }

    @Override
    public void execute() {
        if (intakeBreakSupplier.get() == true) {
            // flash orange WIP
            if (ls.isOn()) {
                ls.setRGB(0, 0, 0); // off
            } else {
                ls.setRGB(255, 165, 0); // orange
            }
        } else if (shooterBreakSupplier.get() == true) {
            // flash green WIP
            if (ls.isOn()) {
                ls.setRGB(0, 0, 0); // off
            } else {
                ls.setRGB(0, 255, 0); // green
            }
        } else {
            if (DriverStation.isEnabled()) {
                Optional<Alliance> ally = DriverStation.getAlliance();
                if (ally != null) {
                    if (ally.get() == Alliance.Red) {
                        ls.setRGB(255, 0, 0); // red
                    } else {
                        ls.setRGB(0, 255, 0); // blue
                    }
                } else {
                    ls.setRGB(255, 255, 0); // yellow
                }
            } else {
                ls.setRGB(255, 255, 0); // yellow
            }
        }
    }
}
