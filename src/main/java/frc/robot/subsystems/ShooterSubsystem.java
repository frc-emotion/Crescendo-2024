package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor, feederMotor;
    private final BangBangController controller;

    private final RelativeEncoder shooterEncoder;
    private final SimpleMotorFeedforward feedforward;
    
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(ShooterConstants.shooterPort, MotorType.kBrushless);

        shooterMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        shooterMotor.setSecondaryCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        shooterMotor.setIdleMode(IdleMode.kCoast);

        feederMotor = new CANSparkMax(ShooterConstants.feederPort, MotorType.kBrushless);

        feederMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        feederMotor.setSecondaryCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        feederMotor.setIdleMode(IdleMode.kBrake);

        shooterEncoder = shooterMotor.getEncoder();

        controller = new BangBangController();
        controller.setTolerance(ShooterConstants.kControllerTolerance);

        feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    }

    /**
     * Sets the target speed of the shooter using feedforward and BangBangController. Controller values
     * multiplied by 12 to change units to volts. Feedforward control is used to sustain speeds and reduce the
     * use of the BangBangController. Feedforward ouput is multiplied by a constant to prevent overspeeding.
     * 
     * @param speed The target speed for the shooter motor from [-1, 1].
     */
    public void setShooterSpeed(double speed) {
        shooterMotor.setVoltage(
            12.0 * controller.calculate(shooterEncoder.getVelocity(), speed) + 
            ShooterConstants.kFeedForwardAdjustment * feedforward.calculate(speed)
        );
    }

    /**
     * Gets the speed of the shooter.
     * 
     * @return The speed of the shooter
     */
    public double getShooterSpeed() {
        return shooterMotor.get();
    }

    /**
     * Gets the voltage supplied to the shooter.
     * 
     * @return The bus voltage to the shooter
     */
    public double getShooterVoltage() {
        return shooterMotor.getBusVoltage();
    }

    /** 
     * Stops supplying power to the shooter.
     */
    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    /**
     * Sets the speed of the feeder.
     * 
     * @param speed The speed to set the feeder to
     */
    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    /**
     * Checks if the shooter has reached the setpoint.
     * 
     * @return If the shooter is at the target speed
     */
    public boolean isShooterReady() {
        return controller.atSetpoint(); // TODO: Check if this method works properly
    }

    /**
     * Stops the feeder
     */
    public void stopFeeder() {
        feederMotor.stopMotor();
    }
}