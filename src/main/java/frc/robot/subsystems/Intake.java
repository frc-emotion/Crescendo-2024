package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    private final CANcoder pivotEncoder; 
    private final CANcoderConfiguration intakeCANConfig;
    private final MagnetSensorConfigs intakeMagnetConfig;

    // private final ArmFeedforward feedforward; NOT USEFUL W/ SPARKPID????
    private final SparkPIDController pivotController;

    public boolean down;
    
    public Intake() {

        down = false; 

        pivotMotor = new CANSparkMax(IntakeConstants.INTAKE_PIVOT_PORT, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_PIVOT_PORT, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setIdleMode(IdleMode.kCoast);
 
        pivotMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        // feedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV);

        pivotController = pivotMotor.getPIDController();
        pivotController.setP(IntakeConstants.kP_PIVOT);
        pivotController.setI(IntakeConstants.kI_PIVOT);
        pivotController.setD(IntakeConstants.kD_PIVOT);
        pivotController.setOutputRange(IntakeConstants.MIN_POSITION, IntakeConstants.MAX_POSITION);

        intakeCANConfig = new CANcoderConfiguration();
        intakeMagnetConfig = new MagnetSensorConfigs();

        intakeMagnetConfig.withMagnetOffset(Units.radiansToDegrees(IntakeConstants.INTAKE_ENCODER_OFFSET));
        intakeMagnetConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        intakeMagnetConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive); 

        pivotEncoder = new CANcoder(0);

        pivotEncoder.getConfigurator().apply(intakeCANConfig);
    }

    public void toggleEndState(){
        down = !down;
    }

    public void pivot(){
        if(down){
            pivotController.setReference(IntakeConstants.INTAKE_DOWN_POSITION, ControlType.kPosition);
        }
        if(!down){
            pivotController.setReference(IntakeConstants.INTAKE_UP_POSITION, ControlType.kPosition);
        }
    }

    public boolean checkCurrentSpike(){
        if(pivotMotor.getOutputCurrent() > IntakeConstants.CURRENT_SPIKE_THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

    public void pivotStop(){
        pivotMotor.set(0);
    }

    public void intakeForward() {
        intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeReverse() {
        intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeStop() {
        intakeMotor.set(0);
    }
}