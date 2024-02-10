package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    //motors on intake (one for pivot and one for the rotation of the wheels)
    private final CANSparkMax pivot;
    private final CANSparkMax wheels;
    //initialize CANcoder
    private final CANcoderConfiguration intakeCANConfig;
    private final MagnetSensorConfigs intakeMagnetConfig;
    private final CANcoder pivotEncoder;
    //PID
    private final SparkPIDController pid;
    //like where it is, pivot wise
    public boolean intaking;

    public Intake() {
        pivot =
            new CANSparkMax(
                IntakeConstants.INTAKE_PIVOT_PORT,
                MotorType.kBrushless
            );
        wheels =
            new CANSparkMax(
                IntakeConstants.INTAKE_PIVOT_PORT,
                MotorType.kBrushless
            );

        wheels.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        wheels.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        wheels.setIdleMode(IdleMode.kCoast);

        pivot.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivot.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivot.setIdleMode(IdleMode.kBrake);

        pid = pivot.getPIDController();
        pid.setP(IntakeConstants.kP_PIVOT);
        pid.setI(IntakeConstants.kI_PIVOT);
        pid.setD(IntakeConstants.kD_PIVOT);
        pid.setOutputRange(
            IntakeConstants.MIN_POSITION,
            IntakeConstants.MAX_POSITION
        );

        intakeCANConfig = new CANcoderConfiguration();
        intakeMagnetConfig = new MagnetSensorConfigs();

        intakeMagnetConfig.withMagnetOffset(
            Units.radiansToDegrees(IntakeConstants.INTAKE_ENCODER_OFFSET)
        );
        intakeMagnetConfig.withAbsoluteSensorRange(
            AbsoluteSensorRangeValue.Unsigned_0To1
        );
        intakeMagnetConfig.withSensorDirection(
            SensorDirectionValue.Clockwise_Positive
        );

        pivotEncoder = new CANcoder(0);

        pivotEncoder.getConfigurator().apply(intakeCANConfig);
    }

    //intake wheels direction
    public void intakeForward() {
        wheels.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void reverseIntake() {
        wheels.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    //copied from preena
    public void toggleEndState() {
        intaking = !intaking;
    }

    public void pivot() {
        if (intaking == true) {
            pid.setReference(
                IntakeConstants.INTAKE_DOWN_POSITION,
                ControlType.kPosition
            );
        }
        if (intaking == false) {
            pid.setReference(
                IntakeConstants.INTAKE_UP_POSITION,
                ControlType.kPosition
            );
        }
    }

    public boolean CurrentSpike() {
        if (
            pivot.getOutputCurrent() > IntakeConstants.CURRENT_SPIKE_THRESHOLD
        ) {
            return true;
        } else {
            return false;
        }
    }

    //stopping stuff
    public void stopPivot() {
        pivot.set(0);
    }

    public void stopIntake() {
        wheels.set(0);
    }
}
