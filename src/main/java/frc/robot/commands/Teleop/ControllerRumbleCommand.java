package frc.robot.commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerRumbleCommand extends Command {
    private final XboxController driverController, operatorController;

    private final Supplier<Boolean> isNoteIntake, isIntakeDown, isShooterAtAmpSpeed, isShooterAtSpeed;

    private double lastRumbleTime;
    private double nextDelay;

    private boolean hasIntakePulsed;

    public ControllerRumbleCommand(
            XboxController driverController,
            XboxController operatorController,
            Supplier<Boolean> isNoteIntake,
            Supplier<Boolean> isIntakeDown,
            Supplier<Boolean> isShooterAtAmpSpeed,
            Supplier<Boolean> isShooterAtSpeed) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.isNoteIntake = isNoteIntake;
        this.isIntakeDown = isIntakeDown;
        this.isShooterAtSpeed = isShooterAtSpeed;
        this.isShooterAtAmpSpeed = isShooterAtAmpSpeed;

        lastRumbleTime = 0;
        nextDelay = 0;
        hasIntakePulsed = false;
    }

    @Override
    public void execute() {
        if (nextDelay == 0 || System.currentTimeMillis() - lastRumbleTime > nextDelay) {

            // If there is a note in the intake and the intake is down, both controllers
            // pulse twice, with 2 cycles of off and on. Each cycle takes 100ms.
            if (isNoteIntake.get() && isIntakeDown.get() && !hasIntakePulsed) {
                if (nextDelay == 0) {
                    rumbleDriver(0.5);
                    rumbleOp(0.5);
                    nextDelay = 50;
                } else if (nextDelay % 2 == 1) {
                    rumbleDriver(0);
                    rumbleOp(0);
                    nextDelay++;
                } else if (nextDelay % 2 == 0) {
                    rumbleDriver(0.5);
                    rumbleOp(0.5);
                    nextDelay++;
                }
                if (nextDelay == 53) {
                    rumbleDriver(0);
                    rumbleOp(0);
                    hasIntakePulsed = true;
                    nextDelay = 0;
                }
            } else if (isShooterAtSpeed.get()) {
                rumbleOp(0.5);
                rumbleDriver(0);
            } else if (isShooterAtAmpSpeed.get()) {
                rumbleOp(0.25);
                rumbleDriver(0);
            } else {
                rumbleDriver(0);
                rumbleOp(0);
            }

            if (!isIntakeDown.get()) {
                hasIntakePulsed = false;
            }
        }

    }

    public void rumbleDriver(double strength) {
        driverController.setRumble(RumbleType.kBothRumble, strength);
    }

    public void rumbleOp(double strength) {
        operatorController.setRumble(RumbleType.kBothRumble, strength);
    }
}
