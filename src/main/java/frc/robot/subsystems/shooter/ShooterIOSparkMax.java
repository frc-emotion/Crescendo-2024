package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

/** The IO Layer for the Shooter Subsystem using CANSparkMax motor controllers. */
public class ShooterIOSparkMax implements ShooterIO {
    private CANSparkMax shooterMotor;
    private RelativeEncoder encoder;
    private SparkPIDController pidController;

    private double target;

    public ShooterIOSparkMax() {
        shooterMotor =
            new CANSparkMax(ShooterConstants.shooterPort, MotorType.kBrushless);

        shooterMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_SMART);
        shooterMotor.setSecondaryCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        shooterMotor.setIdleMode(IdleMode.kCoast);

        shooterMotor.enableVoltageCompensation(7); // tune later

        encoder = shooterMotor.getEncoder();
        encoder.setMeasurementPeriod(16);
        encoder.setAverageDepth(2);

        pidController = shooterMotor.getPIDController();

        pidController.setP(ShooterConstants.SHOOTER_kP);
        pidController.setI(ShooterConstants.SHOOTER_kI);
        pidController.setD(ShooterConstants.SHOOTER_kD);
        pidController.setFF(ShooterConstants.SHOOTER_kFeedForward);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.isAtTarget = isAtTarget();
        inputs.velocity = getVelocity();
        inputs.targetVelocity = target;
    }

    @Override
    public void updateConstants(double kP, double kI, double kD, double kFF) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
    }
    
    @Override
    public void setShooterVelocity(double rpm) {
        target = rpm;
        pidController.setReference(target / 2, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        shooterMotor.set(0);
    }

    @Override
    public void setRaw(double speed) {
        shooterMotor.set(speed);
    }

    public boolean isAtTarget() {
        return (
            Math.abs(encoder.getVelocity() - target) <
            ShooterConstants.kMaxOutputError
        );
    }

    public double getVelocity() {
        return encoder.getVelocity() * 2;
    }

    /** Retrieves the current temperature of the Shooter Motor. Is automatically logged by the AdvantageKit Output Auto Logger. */
    @AutoLogOutput(key = "Shooter/Temperature")
    public double getMotorTemperature() {
        return shooterMotor.getMotorTemperature();
    }

    @AutoLogOutput(key = "Shooter/Voltage")
    public double getShooterVoltage() {
        return shooterMotor.getBusVoltage();
    }

}
