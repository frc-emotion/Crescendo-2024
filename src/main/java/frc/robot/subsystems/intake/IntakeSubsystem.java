package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotDataMode;
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
    
    SendableNumber kP_Drive = new SendableNumber(SubsystemTab.INTAKE, "kP Drive", IntakeConstants.kP_DRIVE);
    SendableNumber kI_Drive = new SendableNumber(SubsystemTab.INTAKE, "kI Drive", IntakeConstants.kI_DRIVE);
    SendableNumber kD_Drive = new SendableNumber(SubsystemTab.INTAKE, "kD Drive", IntakeConstants.kD_DRIVE);
    SendableNumber DriveSpeed = new SendableNumber(SubsystemTab.INTAKE, "Target Drive Speed ", IntakeConstants.DRIVE_SPEED);


    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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

    /** Updates the PID and Speed constants for the Intake IO Layer. */
    public void updateConstants() {
        io.updateConstants(
            kD_Pivot.get(),
            kI_Pivot.get(),
            kI_Pivot.get(),
            PivotSpeed.get(),
            PivotAccel.get(),
            kP_Drive.get(),
            kI_Drive.get(),
            kD_Drive.get()
        );
    }

    /**
     * Toggles Intake up/down
     */
    public void togglePivot() {
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
    public double getPivotPos() {
        return inputs.pivotPos;
    }

    /**
     * Get current encoder position, converted to 0-360 degrees
     * @return Encoder position in degrees
     */
    public double getPivotDegrees() {
        return (
            (inputs.pivotPos / IntakeConstants.GEAR_REDUCTION) *
            360.0
        );
    }

    /**
     * Stop the pivot motors
     */
    public void stopPivot() {
        io.stopPivot();
    }

    /** Stops the drive motor */
    public void stopDrive() {
        io.stopDrive();
    }

    /**
     * Set the goal position for the Pivot motor
     * @param position Position to move pivot motor to (0 to 1)
     */
    public void setPivotTarget(double position) {
        io.setPivotTarget(position);
    }

    /**
     * Travel to Pivot Controller setpoint
     */
    public void goToPivotSetpoint() {
        io.goToPivotSetpoint();
    }

    /**
     * Check if the Pivot Motor has reached the Controller setpoint
     * @return true if setpoint has been reached
     */
    public boolean isPivotAtTarget() {
        return inputs.isPivotAtTarget;
    }

    /**
     * Check if the beam sensor has been broken
     * @return true if beam is unbroken
     */
    public boolean getBeamState() {
        return inputs.beamState;
    }

    // INTAKE MOTORS -------------------------------------------

    public void intakeForward() {
        io.setDriveRaw(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeReverse() {
        io.setDriveRaw(-IntakeConstants.INTAKE_MOTOR_SPEED * (2.0 / 3.0));
    }

    /**
     * Set the intake motor's speed
     * @param speed Intake motor speed (0 to 1)
     */
    public void setIntake(double speed) {
        io.setDriveRaw(speed);
    }

    public void setIntakeVelocity(double velocity) {
        io.setDriveVelocity(velocity);
    }

    /**
     * Get goal position
     * @return Pivot controller goal position
     */
    public double getPivotTarget() {
        return inputs.pivotTarget;
    }

    public double getDriveSpeed() {
        return inputs.driveSpeed;
    }

    public double getDriveTarget() {
        return inputs.targetDriveSpeed;
    }

    public double getDrivePos() {
        return inputs.drivePos;
    }

    /**
     * Initialize Intake Shuffleboard
     */
    private void initShuffleboard() {
        if (Constants.ROBOT_DATA_MODE == RobotDataMode.MATCH) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.INTAKE);
        ShuffleboardLayout pivotData = moduleData.getLayout(
            "Intake Pivot Data",
            BuiltInLayouts.kList
        );

        pivotData.addNumber(
            "Pivot Position Revolutions",
            () -> getPivotPos()
        );
        pivotData.addBoolean("At Setpoint", this::isPivotAtTarget);
        pivotData.addBoolean("Is Down", this::isDown);
        pivotData.addDouble("Pivot Target", this::getPivotTarget);
        pivotData.addNumber("Pivot Position Degrees", this::getPivotDegrees);

        ShuffleboardLayout driveData = moduleData.getLayout("Intake Drive Data", BuiltInLayouts.kList);

        driveData.addBoolean("Beam Break", this::getBeamState);
        driveData.addNumber("Drive Speed", this::getDriveSpeed);
        driveData.addNumber("Drive Target Speed", this::getDriveTarget);
        

        // persianPositions.addDouble("Current", () -> pivotMotor.getOutputCurrent());

        pivotData.withSize(2, 4);
        driveData.withSize(2, 4);
    }
}
