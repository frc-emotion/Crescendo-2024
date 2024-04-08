package frc.robot.commands.Auto.SubsystemCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.other.LEDSubsystem;
import frc.robot.subsystems.other.LEDSubsystem.LEDStyle;

public class LEDAutoCommand extends Command {

    private final LEDSubsystem ls;
    private Color nextColor;
    private LEDStyle style;
    private long timeLastBlink;

    public LEDAutoCommand(LEDSubsystem ls) {
        this.ls = ls;
        this.timeLastBlink = System.currentTimeMillis();

        addRequirements(ls);
    }

    @Override
    public void execute() {
        if (
            DriverStation.isAutonomousEnabled() &&
            DriverStation.getAlliance() != null
        ) {
            nextColor =
                DriverStation.getAlliance().get() == Alliance.Blue
                    ? LEDConstants.BLUE_ALLIANCE_COLOR
                    : LEDConstants.RED_ALLIANCE_COLOR;
            style = LEDStyle.BLINK;
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
