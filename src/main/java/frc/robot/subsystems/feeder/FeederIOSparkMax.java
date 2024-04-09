package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ShooterConstants;

public class FeederIOSparkMax implements FeederIO {
    private CANSparkMax feederMotor;
    private RelativeEncoder encoder;
    private SparkPIDController pidController;
    private DigitalInput beamSensor;

    private double target;

    public FeederIOSparkMax() {
         feederMotor =
            new CANSparkMax(ShooterConstants.feederPort, MotorType.kBrushless);

        feederMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT_SMART);
        feederMotor.setSecondaryCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setInverted(true);

        encoder = feederMotor.getEncoder();

        pidController = feederMotor.getPIDController();
        pidController.setP(ShooterConstants.FEEDER_kP);
        pidController.setI(ShooterConstants.FEEDER_kI);
        pidController.setD(ShooterConstants.FEEDER_kD);
        pidController.setFF(ShooterConstants.FEEDER_kFF);

        beamSensor = new DigitalInput(ShooterConstants.BREAK_SENSOR_PORT);
    }

    @Override
    public void updateInputs(FeederIOInputsAutoLogged inputs) {
        inputs.targetVelocity = getTarget();
        inputs.velocity = getVelocity();
        inputs.isAtTarget = isAtTarget();
        inputs.isBeamBroken = getBeamState();
    }

    @Override
    public void updateConstants(double kP, double kI, double kD, double kFF) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
    }
    
    @Override
    public void set(double speed) {
        feederMotor.set(speed);
    }

    @Override
    public void setVelocity(double velocity) {
        target = velocity;
        pidController.setReference(target, ControlType.kVelocity);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getTarget() {
        return target;
    }

    public boolean isAtTarget() {
        return Math.abs(getVelocity() - getTarget()) < ShooterConstants.FEEDER_MAX_ERROR;
    }

    public boolean getBeamState() {
        return beamSensor.get();
    }

    @Override
    public void stop() {
        feederMotor.set(0);
    }
}
