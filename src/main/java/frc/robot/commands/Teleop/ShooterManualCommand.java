package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class ShooterManualCommand extends Command {

    private final Supplier<Boolean> shooterSupplier, feederSupplier;
    private final ShooterSubsystem shooterSubsystem;

    // private boolean feederState;
    // private boolean hasIndexed;

    public ShooterManualCommand(
            Supplier<Boolean> feederSupplier,
            Supplier<Boolean> shooterSupplier,
            ShooterSubsystem shooterSubsystem) {
        this.feederSupplier = feederSupplier;
        this.shooterSupplier = shooterSupplier;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

        // feederState = false;
        // hasIndexed = false;
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.updatePID();
    }

    @Override
    public void execute() {
        if (shooterSupplier.get()) {
            /*
             * if(feederState) {
             * shooterSubsystem.setFeederSpeed(
             * ShooterConstants.kFeedSpeed
             * );
             * if(hasIndexed) {
             * hasIndexed = false;
             * }
             * } else {
             * shooterSubsystem.stopShooter();
             * }
             * feederState = !feederState;
             */

            shooterSubsystem.setShooterVelocity(ShooterConstants.SHOOTER_SPEED_RPM); // default speed is 4000, amp is 1250
            // shooterSubsystem.setShooterRaw(0.3);

        } else {
            shooterSubsystem.setShooterRaw(0);
            // shooterSubsystem.setShooterVelocity(0);
        }

        if(shooterSubsystem.isAtTarget(ShooterConstants.SHOOTER_SPEED_RPM) && shooterSubsystem.isProjectileFed()) {
            RobotContainer.operatorController_HID.setRumble(RumbleType.kBothRumble, 1);
        } else if(shooterSubsystem.isAtTarget(ShooterConstants.SHOOTER_SPEED_RPM)) {
            RobotContainer.operatorController_HID.setRumble(RumbleType.kBothRumble, 0.25);
        } else {
            RobotContainer.operatorController_HID.setRumble(RumbleType.kBothRumble, 0);
        }

        if (feederSupplier.get()) {
            shooterSubsystem.setFeederSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
        } else {
            shooterSubsystem.stopFeeder();
        }

        /*
         * // Stops the shooter once indexed
         * if(shooterSubsystem.isProjectileFed() && !hasIndexed) {
         * shooterSubsystem.stopFeeder();
         * hasIndexed = true;
         * try {
         * wait(100);
         * } catch(InterruptedException iex) {
         * end(true);
         * }
         * }
         */

        // if (feederSupplier.get() > OIConstants.SHOOTER_DEADZONE) {
        // shooterSubsystem.setFeederSpeed(ShooterConstants.kShootSpeedRotationsPerSecond);
        // } else {
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.stopFeeder();
        shooterSubsystem.stopShooter();
    }
}
