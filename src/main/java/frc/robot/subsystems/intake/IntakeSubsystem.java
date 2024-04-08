package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.Constants.RobotDataMode;
import frc.robot.Constants.RobotLoggingMode;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.SendableNumber;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Intake Subsystem
 */
public class IntakeSubsystem extends SubsystemBase {
    SendableNumber kP_Pivot = new SendableNumber(SubsystemTab.INTAKE, "kP Pivot", IntakeConstants.kP_PIVOT);
    SendableNumber kI_Pivot = new SendableNumber(SubsystemTab.INTAKE, "kI Pivot", IntakeConstants.kI_PIVOT);
    SendableNumber kD_Pivot = new SendableNumber(SubsystemTab.INTAKE, "kD Pivot", IntakeConstants.kD_PIVOT);
    SendableNumber PivotSpeed = new SendableNumber(SubsystemTab.INTAKE, "Max Pivot Speed", IntakeConstants.kMaxVelocity);
    SendableNumber PivotAccel = new SendableNumber(SubsystemTab.INTAKE, "Max Pivot Accel", IntakeConstants.kMaxAccel);

    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    /**
     * Constructs an IntakeSubsystem Instance
     */
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;  
        initShuffleboard();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void updateConstants() {
        io.updateConstants(
            kD_Pivot.get(),
            kI_Pivot.get(),
            kI_Pivot.get(),
            PivotSpeed.get(),
            PivotAccel.get()
        );
    }

    /**
     * Toggles Intake up/down
     */
    public void toggleState() {
        io.toggleIntake();
    }

    /**
     * Check if Intake is down
     * @return true if Intake is down
     */
    public boolean isDown() {
        return inputs.isDown;
    }

    /**
     * Get the current encoder position
     * @return Pivot encoder position
     */
    public double getPosition() {
        return inputs.pivotPos;
    }

    /**
     * Get current encoder position, converted to 0-360 degrees
     * @return Encoder position in degrees
     */
    public double getDegrees() {
        return (
            (inputs.pivotPos / IntakeConstants.GEAR_REDUCTION) *
            360.0
        );
    }

    /**
     * Stop the pivot motor
     */
    public void stopPivot() {
        io.stopPivot();
    }

    public void stopDrive() {
        io.stopDrive();
    }

    /**
     * Set the goal position for the Pivot motor
     * @param position Position to move pivot motor to (0 to 1)
     */
    public void setGoal(double position) {
        io.setPivotTarget(position);
    }

    /**
     * Travel to Pivot Controller setpoint
     */
    public void travelToSetpoint() {
        io.goToSetpoint();
    }

    /**
     * Check if the Pivot Motor has reached the Controller setpoint
     * @return true if setpoint has been reached
     */
    public boolean hasReachedSetpoint() {
        return inputs.atTarget;
    }

    /**
     * Check if the beam sensor has been broken
     * @return true if beam is unbroken
     */
    public boolean getBeamState() {
        return inputs.beamState;
    }

    // TESTING ---------------------------------------------

    public void simplePivot() {
        io.setPivotSpeed(IntakeConstants.INTAKE_PIVOT_SPEED);
    }

    public void revSimplePivot() {
        io.setPivotSpeed(-IntakeConstants.INTAKE_PIVOT_SPEED);
    }

    // INTAKE MOTORS -------------------------------------------

    public void intakeForward() {
        io.setDriveSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeReverse() {
        io.setDriveSpeed(-IntakeConstants.INTAKE_MOTOR_SPEED * (2.0 / 3.0));
    }

    /**
     * Set the intake motor's speed
     * @param speed Intake motor speed (0 to 1)
     */
    public void setIntake(double speed) {
        io.setDriveSpeed(speed);
    }

    /**
     * Get goal position
     * @return Pivot controller goal position
     */
    public double getGoal() {
        return inputs.pivotTarget;
    }

    /**
     * Initialize Intake Shuffleboard
     */
    private void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == RobotDataMode.MATCH) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.INTAKE);
        ShuffleboardLayout persianPositions = moduleData.getLayout(
            "Persian Positions",
            BuiltInLayouts.kList
        );

        persianPositions.addNumber(
            "Pivot Motor Position",
            () -> getPosition()
        );

        persianPositions.addNumber("Pivot Position Degrees", this::getDegrees);
        persianPositions.addBoolean("Beam Break", () -> getBeamState());
        persianPositions.addBoolean("At Setpoint", this::hasReachedSetpoint);
        persianPositions.addBoolean("Is Down", this::isDown);
        persianPositions.addDouble("Current Goal", this::getGoal);

        // persianPositions.addDouble("Current", () -> pivotMotor.getOutputCurrent());

        persianPositions.withSize(2, 4);
    }
}
