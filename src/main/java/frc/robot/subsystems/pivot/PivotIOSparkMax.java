package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class PivotIOSparkMax implements PivotIO {
    private CANSparkMax pivotMotor;
    private SparkPIDController pivotPID;
    private RelativeEncoder relativeEncoder;

    private double target;

    public PivotIOSparkMax() {
        pivotMotor = new CANSparkMax(Constants.PivotConstants.PIVOT_PORT, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(Constants.PivotConstants.MAX_CURRENT_SMART);
        pivotMotor.setSecondaryCurrentLimit(Constants.PivotConstants.MAX_CURRENT);
        pivotMotor.setInverted(true);

        pivotPID = pivotMotor.getPIDController();

        pivotPID.setP(Constants.PivotConstants.PIVOT_KP);
        pivotPID.setI(Constants.PivotConstants.PIVOT_KI);
        pivotPID.setD(Constants.PivotConstants.PIVOT_KD);
        pivotPID.setFF(Constants.PivotConstants.PIVOT_KF);

        pivotPID.setOutputRange(-Constants.PivotConstants.PIVOT_AUTO_SPEED,
                Constants.PivotConstants.PIVOT_AUTO_SPEED);

        relativeEncoder = pivotMotor.getEncoder();
    }

    @Override
    public void updatePID(double kP, double kI, double kD) {
        pivotPID.setP(kP);
        pivotPID.setI(kI);
        pivotPID.setD(kD);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotPos = relativeEncoder.getPosition();
        inputs.pivotAngularSpeed = relativeEncoder.getVelocity();
        inputs.target = getTarget();
        inputs.atTargetAngle = isAtTarget();
    }

    @Override
    public void resetPosition(double position) {
        relativeEncoder.setPosition(position);
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        pivotPID.setReference(target, ControlType.kPosition);
    }

    @Override
    public void stop() {
        pivotMotor.set(0);
    }

    @Override
    public void set(double speed) {
        pivotMotor.set(speed);
    }

    public boolean isAtTarget(double target) {
        return Math.abs(target - relativeEncoder.getPosition()) < PivotConstants.MAX_ERROR;
    }
    
    public boolean isAtTarget() {
        return isAtTarget(this.target);
    }

    public double getTarget() {
        return target;
    }

    @AutoLogOutput(key = "Pivot/Current")
    public double getCurrent() {
        return pivotMotor.getOutputCurrent();
    }
}

