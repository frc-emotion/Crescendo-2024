package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
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

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax intakeMotor;

    private GenericEntry kIEntry, kDEntry, kPEntry, kIEntry2, kDEntry2, kPEntry2;

    private RelativeEncoder leftEncoder, rightEncoder, driveEncoder;

    private final DigitalInput breakSensor;

    // private final SparkPIDController pivotController;
    private final ProfiledPIDController pivotControllerLeft, pivotControllerRight;

    public boolean down;

    /**
     * Constructs an IntakeSubsystem Instance
     */
    public IntakeSubsystem() {
        down = false;

        leftMotor =
            new CANSparkMax(
                IntakeConstants.INTAKE_PIVOT_PORT_LEFT,
                MotorType.kBrushless
            );
        rightMotor =
            new CANSparkMax(
                IntakeConstants.INTAKE_PIVOT_PORT_RIGHT,
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

        leftMotor.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        leftMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setInverted(true);

        rightMotor.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        rightMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        rightMotor.setIdleMode(IdleMode.kBrake);
        //rightMotor.setInverted(true);

        //rightMotor.follow(leftMotor, true);
        //leftMotor.follow(rightMotor, true);

        pivotControllerLeft =
            new ProfiledPIDController(
                IntakeConstants.kP_PIVOT,
                IntakeConstants.kI_PIVOT,
                IntakeConstants.kD_PIVOT,
                new TrapezoidProfile.Constraints(
                    IntakeConstants.kMaxVelocity,
                    IntakeConstants.kMaxAccel
                )
            );

        pivotControllerRight = 
            new ProfiledPIDController(
                IntakeConstants.kPR_PIVOT,
                IntakeConstants.kIR_PIVOT,
                IntakeConstants.kDR_PIVOT,
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

        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(
            1.0 / IntakeConstants.GEAR_REDUCTION
        );

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(
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
            leftMotor.getOutputCurrent() >
            IntakeConstants.CURRENT_SPIKE_THRESHOLD
        );
    }

    /**
     * Get the current encoder position
     * @return Pivot encoder position
     */
    public double getPositionLeft() {
        return leftEncoder.getPosition();
    }

    public double getPositionRight() {
        return rightEncoder.getPosition();
    }

    /**
     * Get current encoder position, converted to 0-360 degrees
     * @return Encoder position in degrees
     */
    public double getDegrees(boolean left) {
        if (left) {
            return (
            (getPositionLeft() / IntakeConstants.GEAR_REDUCTION) *
            360.0
        );
        }
        return (
            (getPositionRight() / IntakeConstants.GEAR_REDUCTION) *
            360.0
        );
    }

    /**
     * Stop the pivot motor
     */
    public void pivotStop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    /**
     * Set the goal position for the Pivot motor
     * @param position Position to move pivot motor to (0 to 1)
     */
    public void setGoal(double leftPosition, double rightPosition) {
        pivotControllerLeft.setGoal(leftPosition);

        pivotControllerRight.setGoal(rightPosition);
    }

    /**
     * Travel to Pivot Controller setpoint
     */
    public void travelToSetpoint() {
        leftMotor.set(pivotControllerLeft.calculate(getPositionLeft()));
        rightMotor.set(pivotControllerRight.calculate(getPositionRight()));
        
    }

    /**
     * Check if the Pivot Motor has reached the Controller setpoint
     * @return true if setpoint has been reached
     */
    public boolean hasReachedSetpoint() {
        return pivotControllerLeft.atGoal() && pivotControllerRight.atGoal();
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
        leftMotor.set(0.15);
        rightMotor.set(0.15);
        
    }

    public void stopSimple() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    public void revSimplePivot() {
        leftMotor.set(-0.15);
        rightMotor.set(-0.15);
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
    public double getGoal(boolean leftController) {
        if (leftController) {
            return pivotControllerLeft.getGoal().position;
        }
        
        return pivotControllerRight.getGoal().position;
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
        return leftMotor.getMotorTemperature();
    }

    public void updatePID() {
        if(Constants.DEBUG_MODE_ACTIVE) {
            this.pivotControllerLeft.setI(this.kIEntry.getDouble(IntakeConstants.kI_PIVOT));
            this.pivotControllerLeft.setD(this.kDEntry.getDouble(IntakeConstants.kD_PIVOT));
            this.pivotControllerLeft.setP(this.kPEntry.getDouble(IntakeConstants.kP_PIVOT));

            this.pivotControllerRight.setI(this.kIEntry2.getDouble(IntakeConstants.kIR_PIVOT));
            this.pivotControllerRight.setD(this.kDEntry2.getDouble(IntakeConstants.kDR_PIVOT));
            this.pivotControllerRight.setP(this.kPEntry2.getDouble(IntakeConstants.kPR_PIVOT));
        } else {
            pivotControllerLeft.setPID(
                IntakeConstants.kP_PIVOT,
                IntakeConstants.kI_PIVOT,
                IntakeConstants.kD_PIVOT
            );

            pivotControllerRight.setPID(
                IntakeConstants.kPR_PIVOT,
                IntakeConstants.kIR_PIVOT,
                IntakeConstants.kDR_PIVOT
            );
        }

            
       
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
            "Left Pivot Motor Position",
            () -> leftEncoder.getPosition()
        );
        persianPositions.addNumber(
            "Right Pivot Motor Position",
            () -> rightEncoder.getPosition()
        );

        persianPositions.addNumber("Current Intake Motor", () -> intakeMotor.getOutputCurrent());
        persianPositions.addNumber("Voltage Intake Motor", () -> intakeMotor.getBusVoltage());

        persianPositions.addNumber("Pivot Position Degrees Left", () -> getDegrees(true));
        persianPositions.addNumber("Pivot Position Degrees Right", () -> getDegrees(false));
        persianPositions.addBoolean("Beam Break", () -> getBeamState());
        persianPositions.addBoolean("At Setpoint", this::hasReachedSetpoint);
        persianPositions.addBoolean("Is Up", this::isUp);
        persianPositions.addDouble("Current Goal Left", () -> getGoal(true));
        persianPositions.addDouble("Current Goal Right", () -> getGoal(false));
        persianPositions.addDouble("Current Left Input", () -> leftMotor.get());
        persianPositions.addDouble("Current Right Input", () -> rightMotor.get());

        this.kPEntry = moduleData.add("kP", IntakeConstants.kP_PIVOT).getEntry();

        this.kIEntry = moduleData.add("kI", IntakeConstants.kI_PIVOT).getEntry();

        this.kDEntry = moduleData.add("kD", IntakeConstants.kD_PIVOT).getEntry();

        this.kPEntry2 = moduleData.add("kP2", IntakeConstants.kPR_PIVOT).getEntry();

        this.kIEntry2 = moduleData.add("kI2", IntakeConstants.kIR_PIVOT).getEntry();

        this.kDEntry2 = moduleData.add("kD2", IntakeConstants.kDR_PIVOT).getEntry();

        // moduleData
        //         .add("Left Up", new InstantCommand() {
        //             public void execute() {
        //                 leftMotor.set(0.25);
        //             };

        //             public void end(boolean interrupted) {
        //                 rightMotor.set(0);
        //             };
        //         })
        //         .withWidget(BuiltInWidgets.kCommand)
        //         .withSize(2, 2);
        // moduleData
        //         .add("Left Down", new InstantCommand() {
        //             public void execute() {
        //                 leftMotor.set(-0.25);
        //             };

        //             public void end(boolean interrupted) {
        //                 leftMotor.set(0);
        //             };
        //         })
        //         .withWidget(BuiltInWidgets.kCommand)
        //         .withSize(2, 2);
        // moduleData
        //         .add("Right Up", new InstantCommand() {
        //             public void execute() {
        //                 rightMotor.set(0.25);
        //             };

        //             public void end(boolean interrupted) {
        //                 rightMotor.set(0);
        //             };
        //         })
        //         .withWidget(BuiltInWidgets.kCommand)
        //         .withSize(2, 2);
        // moduleData
        //         .add("Right Down", new InstantCommand() {
        //             public void execute() {
        //                 rightMotor.set(-0.25);
        //             };

        //             public void end(boolean interrupted) {
        //                 rightMotor.set(0);
        //             };
        //         })
        //         .withWidget(BuiltInWidgets.kCommand)
        //         .withSize(2, 2);

        // persianPositions.addDouble("Current", () -> leftMotor.getOutputCurrent());

        persianPositions.withSize(2, 4);
    }
}
