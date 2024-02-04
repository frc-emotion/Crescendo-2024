package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotSubsystem extends SubsystemBase {
    

    private CANSparkMax pivotMotor;
    private SparkPIDController pivotPID;

    private CANcoder absoluteEncoder;
    private MagnetSensorConfigs magnetConfiguration;

    private DigitalInput lowerLimit;

    public PivotSubsystem() {

        this.pivotMotor = new CANSparkMax(Constants.PivotConstants.PIVOT_PORT, MotorType.kBrushless);
        this.lowerLimit = new DigitalInput(Constants.PivotConstants.LOWER_LIMIT_PORT);

        // magnetConfiguration = new MagnetSensorConfigs();

        // magnetConfiguration.withMagnetOffset(Units.radiansToDegrees(absoluteEncoderOffsetRad));
        // magnetConfiguration.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        // magnetConfiguration.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        absoluteEncoder = new CANcoder(Constants.PivotConstants.absoluteEncoderID);

        //absoluteEncoder.getConfigurator().apply(magnetConfiguration);

        pivotPID = pivotMotor.getPIDController();

        pivotPID.setP(Constants.PivotConstants.PIVOT_KP);
        pivotPID.setI(Constants.PivotConstants.PIVOT_KI);
        pivotPID.setD(Constants.PivotConstants.PIVOT_KD);
        pivotPID.setFF(Constants.PivotConstants.PIVOT_KF);

        pivotPID.setOutputRange(-Constants.PivotConstants.PIVOT_AUTO_SPEED - 0.05,
                Constants.PivotConstants.PIVOT_AUTO_SPEED);

    }

    public boolean getLimit() {
        return lowerLimit.get();
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void calibrate() {
        pivotMotor.set(0);
        pivotMotor.getEncoder().setPosition(0);
    }

    public double getRev() {
        return -absoluteEncoder.getPosition().getValueAsDouble();
    }

    public void stop() {
        pivotMotor.set(0);
    }

    public void setAgainst() {
        setRev(79); // TODO: Needs to be changed
    }

    public void setLine() {
        setRev(16.35); // TODO: Needs to be changed
    }

    public void setWheel() {
        setRev(33); // TODO: Needs to be changed
    }

    public void setTrench() {
        setRev(7.5); // TODO: Needs to be changed
    }

    public void setRev(double rev) {
        double target = rev;
        if (rev < Constants.PivotConstants.PIVOT_ZERO_THRESHOLD) {
            target = 0;
        }
        if (rev > Constants.PivotConstants.PIVOT_MAX_REVOLUTION) {
            target = Constants.PivotConstants.PIVOT_MAX_REVOLUTION;
        }
        pivotPID.setReference(-target, ControlType.kPosition);
    }

}
