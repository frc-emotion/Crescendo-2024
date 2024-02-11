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
    
    private int index = 0;

    public PivotSubsystem() {

        this.pivotMotor = new CANSparkMax(Constants.PivotConstants.PIVOT_PORT, MotorType.kBrushless);
        this.lowerLimit = new DigitalInput(Constants.PivotConstants.LOWER_LIMIT_PORT);

        magnetConfiguration = new MagnetSensorConfigs();

        //magnetConfiguration.withMagnetOffset(Units.radiansToDegrees(absoluteEncoderOffsetRad));
        magnetConfiguration.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        magnetConfiguration.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        absoluteEncoder = new CANcoder(Constants.PivotConstants.absoluteEncoderID);

        absoluteEncoder.getConfigurator().apply(magnetConfiguration);

        pivotPID = pivotMotor.getPIDController();

        pivotPID.setP(Constants.PivotConstants.PIVOT_KP);
        pivotPID.setI(Constants.PivotConstants.PIVOT_KI);
        pivotPID.setD(Constants.PivotConstants.PIVOT_KD);
        pivotPID.setFF(Constants.PivotConstants.PIVOT_KF);

        pivotPID.setOutputRange(-Constants.PivotConstants.PIVOT_AUTO_SPEED - 0.05,
                Constants.PivotConstants.PIVOT_AUTO_SPEED);

    }

    public int getIndex() {
        return index;
    }

    public void addIndex() {
        if (Constants.PivotConstants.PIVOT_POSITIONS.length - 1 != index) {
            index++;
        }
    }

    public void subtractIndex() {
        if (index != 0) {
            index--;
        }
    }

    public void setIndex(int i) {
        index = i;
    }

    public void goToPreset() {
        setRev(Constants.PivotConstants.PIVOT_POSITIONS[index]);
    }

    public boolean getLimit() {
        return lowerLimit.get();
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void calibrate() {
        pivotMotor.set(0);
        pivotMotor.getEncoder().setPosition(getRev());
    }

    public double getRev() {
        return -absoluteEncoder.getPosition().getValueAsDouble() * 180.0;
    }

    public void stop() {
        pivotMotor.set(0);
    }

    public void setRev(double rev) {
        double target = rev;
        if (rev < Constants.PivotConstants.PIVOT_MIN_REVOLUTION) {
            target = 0;
        }
        if (rev > Constants.PivotConstants.PIVOT_MAX_REVOLUTION) {
            target = Constants.PivotConstants.PIVOT_MAX_REVOLUTION;
        }
        pivotPID.setReference(-target, ControlType.kPosition);
    }

}
