package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;
import java.util.function.Supplier;

public class LEDCommand extends Command {

    private final LEDSubsystem ls;
    private final Supplier<Boolean> intakeBreakSupplier;
    private final Supplier<Boolean> shooterBreakSupplier;
    private long timeLastBlink;

    private static final long DELAY = 200;

    public LEDCommand(
        LEDSubsystem ls,
        Supplier<Boolean> intakeBreakSupplier,
        Supplier<Boolean> shooterBreakSupplier
    ) {
        this.ls = ls;
        this.intakeBreakSupplier = intakeBreakSupplier;
        this.shooterBreakSupplier = shooterBreakSupplier;
        this.timeLastBlink = System.currentTimeMillis();

        addRequirements(ls);
    }

    @Override
    public void execute() {
        if (intakeBreakSupplier.get() == false) {
            if (System.currentTimeMillis() - timeLastBlink > (DELAY / 2)) {
                if (ls.isOn()) ls.setRGB(0, 0, 0); // off
                else ls.setRGB(255, 40, 0); // orange
                timeLastBlink = System.currentTimeMillis();
            }
            
        } else if (shooterBreakSupplier.get() == false) {
            if (System.currentTimeMillis() - timeLastBlink > (DELAY / 2)) {
                if (ls.isOn()) ls.setRGB(0, 0, 0); // off
                else ls.setRGB(0, 255, 0); // green
                timeLastBlink = System.currentTimeMillis();
            }
        } else if (DriverStation.isEnabled()) {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally != null) {
                if (ally.get() == Alliance.Red) ls.setRGB(255, 0, 0); // red
                else ls.setRGB(0, 0, 255); // blue
            }
        } else ls.setRGB(255, 255, 0); // yellow
    }
}
