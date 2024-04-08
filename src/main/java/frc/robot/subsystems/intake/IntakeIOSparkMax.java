package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax pivotMotor;
    private CANSparkMax pivotMotor2;
    private CANSparkMax intakeMotor;

    private RelativeEncoder pivotEncoder, pivotEncoder2, driveEncoder;

    private DigitalInput breakSensor;

    private final ProfiledPIDController pivotController;

    private boolean isDown = false;
    private double pivotTarget = 0;

    public IntakeIOSparkMax() {
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

        pivotController = new ProfiledPIDController(
            IntakeConstants.kP_PIVOT,
            IntakeConstants.kI_PIVOT,
            IntakeConstants.kD_PIVOT,
            new TrapezoidProfile.Constraints(
                IntakeConstants.kMaxVelocity,
                IntakeConstants.kMaxAccel
            )
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.beamState = getBeamState();
        inputs.isDown = isDown;
        inputs.pivotPos = pivotEncoder.getPosition();
        inputs.pivotDeg = inputs.pivotPos * IntakeConstants.GEAR_REDUCTION * 360;
        inputs.pivotTarget = pivotController.getGoal().position;
        inputs.atTarget = isAtTarget();
    }

    @Override
    public void setDriveSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setPivotTarget(double target) {
        pivotController.setGoal(target);
    }

    @Override
    public void goToSetpoint() {
        setPivotSpeed(pivotController.calculate(getPivotPos()));
    }

    @Override
    public void updateConstants(double kP, double kI, double kD, double maxSpeed, double maxAccel) {
        pivotController.setPID(kP, kI, kD);
        pivotController.setConstraints(new TrapezoidProfile.Constraints(maxSpeed, maxAccel));
    }

    public boolean isAtTarget() {
        return pivotController.atGoal();
    }
    
    public double getTarget() {
        return pivotTarget;
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
