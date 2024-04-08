package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax pivotMotor;
    private CANSparkMax pivotMotor2;
    private CANSparkMax intakeMotor;

    private RelativeEncoder pivotEncoder, pivotEncoder2, driveEncoder;

    private DigitalInput breakSensor;

    private boolean isDown;

    public IntakeIOSparkMax() {
        switch (Constants.ROBOT_LOGGING_MODE) {
            case REAL:
                pivotMotor =
                    new CANSparkMax(
                        IntakeConstants.INTAKE_PIVOT_PORT,
                        MotorType.kBrushless
                    );
                pivotMotor2 =
                    new CANSparkMax(
                        IntakeConstants.INTAKE_PIVOT_2_PORT,
                        MotorType.kBrushless
                    );
                intakeMotor =
                    new CANSparkMax(
                        IntakeConstants.INTAKE_MOTOR_PORT,
                        MotorType.kBrushless
                    );
                breakSensor = new DigitalInput(IntakeConstants.BEAM_BREAKER_PORT);
                break;
            case SIM, REPLAY:
                throw new UnsupportedOperationException("Sim and Replay modes are currently unsupported");
            
        }

        intakeMotor.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        intakeMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        pivotMotor.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        pivotMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotMotor2.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        pivotMotor2.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor2.setIdleMode(IdleMode.kBrake);

        pivotMotor2.follow(pivotMotor, true);

        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder2 = pivotMotor2.getEncoder();
        driveEncoder = intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.beamState = getBeamState();
        inputs.isDown = isDown;
        inputs.pivotPos = pivotEncoder.getPosition();
    }

    public double getPivotPos() {
        return pivotEncoder.getPosition();
    }

    public boolean getBeamState() {
        return breakSensor.get();
    }

    public boolean isDown() {
        return isDown;
    }

    @Override
    public void toggleIntake() {
        isDown = !isDown;
    }

    @Override
    public void setIntakeDownState(boolean isDown) {
        this.isDown = isDown;
    }

    @Override
    public void resetPivotPos(double pos) {
        pivotEncoder.setPosition(pos);
        pivotEncoder2.setPosition(pos);
    }
}
