package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStyle;
import java.util.function.Supplier;

public class LEDCommand extends Command {

    private final LEDSubsystem ls;
    private final Supplier<Boolean> intakeBreakSupplier, intakeDeploySupplier, shooterBreakSupplier, shooterAtTargetSupplier;
    private long timeLastBlink;

    private Color nextColor;
    private LEDStyle style;

    public LEDCommand(
        LEDSubsystem ls,
        Supplier<Boolean> intakeBreakSupplier,
        Supplier<Boolean> intakeDeploySupplier,
        Supplier<Boolean> shooterBreakSupplier,
        Supplier<Boolean> shooterAtTargetSupplier
    ) {
        this.ls = ls;
        this.intakeBreakSupplier = intakeBreakSupplier;
        this.intakeDeploySupplier = intakeDeploySupplier;
        this.shooterBreakSupplier = shooterBreakSupplier;
        this.shooterAtTargetSupplier = shooterAtTargetSupplier;
        this.timeLastBlink = System.currentTimeMillis();

        addRequirements(ls);
    }

    @Override
    public void execute() {
        System.out.println(shooterAtTargetSupplier.get());
        
        if (!intakeBreakSupplier.get()) {
            nextColor = LEDConstants.INTAKE_COLOR;
            style = LEDStyle.SOLID;
        } else if (intakeDeploySupplier.get()) {
            nextColor = LEDConstants.INTAKE_COLOR;
            style = LEDStyle.BLINK_SLOW;
        } else if (shooterAtTargetSupplier.get()) {
            nextColor = LEDConstants.SHOOTER_COLOR;
            style = LEDStyle.BLINK;
        } else if (shooterBreakSupplier.get()) {
            nextColor = LEDConstants.SHOOTER_COLOR;
            style = LEDStyle.SOLID;
        } else if (
            DriverStation.isEnabled() && DriverStation.getAlliance() != null
        ) {
            nextColor =
                DriverStation.getAlliance().get() == Alliance.Blue
                    ? LEDConstants.BLUE_ALLIANCE_COLOR
                    : LEDConstants.RED_ALLIANCE_COLOR;
            style = LEDStyle.SOLID;
        } else {
            nextColor = LEDConstants.DISABLED_COLOR;
            style = LEDStyle.SOLID;
        }

        int delay = LEDStyle.getDelayAmount(style);

        if (delay == 0) {
            solid();
        } else {
            blinkMode(delay);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private void blinkMode(int delay) {
        if (System.currentTimeMillis() - timeLastBlink > delay) {
            if (ls.isOn()) {
                ls.turnOff();
            } else {
                ls.setColor(nextColor);
            }
            timeLastBlink = System.currentTimeMillis();
        }
    }

    private void solid() {
        ls.setColor(nextColor);
    }
}
