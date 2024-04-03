package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Intake Subsystem
 */
public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax pivotMotor2;
    private final CANSparkMax intakeMotor;

    private RelativeEncoder pivotEncoder, pivotEncoder2, driveEncoder;

    private final DigitalInput breakSensor;

    // private final SparkPIDController pivotController;
    private final ProfiledPIDController pivotController;

    public boolean down;

    /**
     * Constructs an IntakeSubsystem Instance
     */
    public IntakeSubsystem() {
        down = false;

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

        pivotController =
            new ProfiledPIDController(
                IntakeConstants.kP_PIVOT,
                IntakeConstants.kI_PIVOT,
                IntakeConstants.kD_PIVOT,
                new TrapezoidProfile.Constraints(
                    IntakeConstants.kMaxVelocity,
                    IntakeConstants.kMaxAccel
                )
            );
        // pivotController.setTolerance(IntakeConstants.kMaxError);

        breakSensor = new DigitalInput(IntakeConstants.BEAM_BREAKER_PORT);

        // pivotController.setP(IntakeConstants.kP_PIVOT);
        // pivotController.setI(IntakeConstants.kI_PIVOT);
        // pivotController.setD(IntakeConstants.kD_PIVOT);
        // pivotController.setOutputRange(IntakeConstants.MIN_POSITION,
        // IntakeConstants.MAX_POSITION);

        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPositionConversionFactor(
            1.0 / IntakeConstants.GEAR_REDUCTION
        );

        pivotEncoder2 = pivotMotor2.getEncoder();
        pivotEncoder2.setPositionConversionFactor(
            1.0 / IntakeConstants.GEAR_REDUCTION
        );

        driveEncoder = intakeMotor.getEncoder();

        initShuffleboard();
    }

    /**
     * Toggles Intake up/down
     */
    public void toggleState() {
        down = !down;
    }

    /**
     * Check if Intake is up
     * @return true if Intake is up
     */
    public boolean isUp() {
        return !down;
    }

    /**
     * Check if Intake is down
     * @return true if Intake is down
     */
    public boolean isDown() { // this method seems redundant ...
        return down;
    }

    /**
     * Check if the current has spiked above the threshold
     * @return true if current has spiked
     */
    public boolean checkCurrentSpike() {
        return (
            pivotMotor.getOutputCurrent() >
            IntakeConstants.CURRENT_SPIKE_THRESHOLD
        );
    }

    /**
     * Get the current encoder position
     * @return Pivot encoder position
     */
    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * Get current encoder position, converted to 0-360 degrees
     * @return Encoder position in degrees
     */
    public double getDegrees() {
        return (
            (pivotEncoder.getPosition() / IntakeConstants.GEAR_REDUCTION) *
            360.0
        );
    }

    /**
     * Stop the pivot motor
     */
    public void pivotStop() {
        pivotMotor.set(0);
    }

    /**
     * Set the goal position for the Pivot motor
     * @param position Position to move pivot motor to (0 to 1)
     */
    public void setGoal(double position) {
        pivotController.setGoal(position);
    }

    /**
     * Travel to Pivot Controller setpoint
     */
    public void travelToSetpoint() {
        pivotMotor.set(pivotController.calculate(getPosition()));
    }

    /**
     * Check if the Pivot Motor has reached the Controller setpoint
     * @return true if setpoint has been reached
     */
    public boolean hasReachedSetpoint() {
        return pivotController.atGoal();
    }

    /**
     * Check if the beam sensor has been broken
     * @return true if beam is unbroken
     */
    public boolean getBeamState() {
        return breakSensor.get();
    }

    // TESTING ---------------------------------------------

    public void simplePivot() {
        pivotMotor.set(IntakeConstants.INTAKE_PIVOT_SPEED);
    }

    public void stopSimple() {
        pivotMotor.set(0.0);
    }

    public void revSimplePivot() {
        pivotMotor.set(-IntakeConstants.INTAKE_PIVOT_SPEED);
    }

    // INTAKE MOTORS -------------------------------------------

    public void intakeForward() {
        intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeReverse() {
        intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED * (2.0 / 3.0));
    }

    /**
     * Set the intake motor's speed
     * @param speed Intake motor speed (0 to 1)
     */
    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Stop the intake motor
     */
    public void intakeStop() {
        intakeMotor.set(0);
    }

    /**
     * Get goal position
     * @return Pivot controller goal position
     */
    public double getGoal() {
        return pivotController.getGoal().position;
    }

    /**
     * Get intake motor's temperature
     * @return Intake motor temperature in Celsius
     */
    public double getIntakeDriveTemp() {
        return intakeMotor.getMotorTemperature();
    }

    /**
     * Get pivot motor's temperature
     * @return Pivot motor temperature in Celsius
     */
    public double getIntakePivotTemp() {
        return pivotMotor.getMotorTemperature();
    }

    /**
     * Initialize Intake Shuffleboard
     */
    private void initShuffleboard() {
        if (!Constants.DEBUG_MODE_ACTIVE) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.INTAKE);
        ShuffleboardLayout persianPositions = moduleData.getLayout(
            "Persian Positions",
            BuiltInLayouts.kList
        );

        persianPositions.addNumber(
            "Intake Motor Position",
            () -> driveEncoder.getPosition()
        );

        persianPositions.addNumber(
            "Pivot Motor Position",
            () -> pivotEncoder.getPosition()
        );
        persianPositions.addNumber(
            "Pivot Motor 2 Position",
            () -> pivotEncoder2.getPosition()
        );
        persianPositions.addNumber("Pivot Position Degrees", this::getDegrees);
        persianPositions.addBoolean("Beam Break", () -> getBeamState());
        persianPositions.addBoolean("At Setpoint", this::hasReachedSetpoint);
        persianPositions.addBoolean("Is Up", this::isUp);
        persianPositions.addDouble("Current Goal", this::getGoal);

        // persianPositions.addDouble("Current", () -> pivotMotor.getOutputCurrent());

        persianPositions.withSize(2, 4);
    }
}
