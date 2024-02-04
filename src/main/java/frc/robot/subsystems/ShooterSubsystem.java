package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor, feederMotor;
    private final SparkPIDController controller;

    private final RelativeEncoder shooterEncoder;
    
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

        controller = shooterMotor.getPIDController();
        controller.setP(ShooterConstants.kP);
        controller.setI(ShooterConstants.kI);
        controller.setD(ShooterConstants.kD);
        controller.setFF(ShooterConstants.kFeedForward);

        controller.setFeedbackDevice(shooterEncoder);
        controller.setOutputRange(ShooterConstants.kMaxOutput, ShooterConstants.kMinOutput);
        controller.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        controller.setSmartMotionMaxVelocity(ShooterConstants.kMaxSpeedRotationsPerSecond * 60, 0);
        controller.setSmartMotionMaxAccel(ShooterConstants.kMaxSpeedRotationsPerSecondSquared * 60, 0);
        controller.setSmartMotionAllowedClosedLoopError(ShooterConstants.kMaxOutputError, 0);
    }

    /**
     * Sets the target speed of the shooter in rotations per second. 
     * Multiplies by 60 because the setReference method for velocity takes in rotations per minute.
     * 
     * @param speed The target speed for the shooter motor in rotations per second.
     */
    public void setShooterVelocity(double speed) {
        controller.setReference(
            speed * 60,
            ControlType.kSmartVelocity
        );
    }

    /**
     * Gets the speed of the shooter.
     * 
     * @return The speed of the shooter
     */
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
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
     * Stops the feeder
     */
    public void stopFeeder() {
        feederMotor.set(0);
    }
}