package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

/** The IO Layer for the Intake Subsystem using CANSparkMax motor controllers. */
public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax pivotMotor;
    private CANSparkMax pivotMotor2;
    private CANSparkMax intakeMotor;

    private RelativeEncoder pivotEncoder, pivotEncoder2, driveEncoder;

    private DigitalInput breakSensor;
    private Debouncer debouncer;

    private final ProfiledPIDController pivotController;
    private final SparkPIDController driveController;

    private boolean isDown = false;
    private double pivotTarget = 0.0;
    private double driveTarget = 0.0;

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

        debouncer = new Debouncer(IntakeConstants.DEBOUNCE_TIME, DebounceType.kBoth);

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

        driveEncoder = intakeMotor.getEncoder();
        driveController = intakeMotor.getPIDController();

        driveController.setP(IntakeConstants.kP_DRIVE);
        driveController.setI(IntakeConstants.kI_DRIVE);
        driveController.setD(IntakeConstants.kD_DRIVE);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.beamState = getBeamState();
        inputs.isDown = isDown;
        inputs.pivotPos = pivotEncoder.getPosition();
        inputs.pivotDeg = inputs.pivotPos * IntakeConstants.GEAR_REDUCTION * 360;
        inputs.pivotTarget = pivotController.getGoal().position;
        inputs.isPivotAtTarget = isPivotAtTarget();
        inputs.driveSpeed = getDriveSpeed();
        inputs.targetDriveSpeed = getDriveTarget();
        inputs.isDriveAtTarget = isDriveAtTarget();
    }

    @Override
    public void setDriveRaw(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setPivotRaw(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setPivotTarget(double target) {
        pivotController.setGoal(target);
    }

    @Override
    public void goToPivotSetpoint() {
        setPivotRaw(pivotController.calculate(getPivotPos()));
    }

    @Override
    public void updateConstants(
        double kP_Pivot, 
        double kI_Pivot, 
        double kD_Pivot, 
        double maxPivotSpeed, 
        double maxPivotAccel,
        double kP_Drive,
        double kI_Drive,
        double kD_Drive
    ) {
        pivotController.setPID(kP_Pivot, kI_Pivot, kD_Pivot);
        pivotController.setConstraints(new TrapezoidProfile.Constraints(maxPivotSpeed, maxPivotAccel));

        driveController.setP(kP_Drive);
        driveController.setI(kI_Drive);
        driveController.setD(kD_Drive);
    }

    @Override
    public void setDriveVelocity(double target) {
        driveController.setReference(target, ControlType.kVelocity);
    }

    public boolean isPivotAtTarget() {
        return pivotController.atGoal();
    }
    
    public double getPivotTarget() {
        return pivotTarget;
    }

    public double getPivotPos() {
        return pivotEncoder.getPosition();
    }

    public boolean getBeamState() {
        return debouncer.calculate(breakSensor.get());
    }

    public boolean isDown() {
        return isDown;
    }

    public double getDriveTarget() {
        return driveTarget;
    }

    public double getDriveSpeed() {
        return driveEncoder.getVelocity();
    }

    public boolean isDriveAtTarget() {
        return Math.abs(getDriveSpeed() - driveTarget) < IntakeConstants.MAX_DRIVE_SPEED_ERROR;
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

    @Override
    public void stop() {
        stopDrive();
        stopPivot();
    }

    @Override
    public void stopDrive() {
        intakeMotor.set(0);
    }

    @Override
    public void stopPivot() {
        pivotMotor.set(0);
    }
}
