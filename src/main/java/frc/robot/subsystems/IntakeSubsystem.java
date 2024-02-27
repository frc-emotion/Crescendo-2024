package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax pivotMotor2;
    private final CANSparkMax intakeMotor;

    private final DigitalInput breakSensor;

    //private final SparkPIDController pivotController;
    private final ProfiledPIDController pivotController;

    public boolean down;
    
    public IntakeSubsystem() {

        down = false; 

        pivotMotor = new CANSparkMax(IntakeConstants.INTAKE_PIVOT_PORT, MotorType.kBrushless);
        pivotMotor2 = new CANSparkMax(IntakeConstants.INTAKE_PIVOT_2_PORT, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        intakeMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setIdleMode(IdleMode.kCoast);
 
        pivotMotor.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        pivotMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotMotor2.setSmartCurrentLimit(IntakeConstants.SMART_MAX_CURRENT);
        pivotMotor2.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor2.setIdleMode(IdleMode.kBrake);
        pivotMotor2.setInverted(true);

        pivotMotor2.follow(pivotMotor);

        pivotController = new ProfiledPIDController(
            IntakeConstants.kP_PIVOT, 
            IntakeConstants.kI_PIVOT, 
            IntakeConstants.kD_PIVOT, 
            new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAccel)
        );
        pivotController.setTolerance(IntakeConstants.kMaxError);

        breakSensor = new DigitalInput(IntakeConstants.BEAM_BREAKER_PORT);
        
        
        // pivotController.setP(IntakeConstants.kP_PIVOT);
        // pivotController.setI(IntakeConstants.kI_PIVOT);
        // pivotController.setD(IntakeConstants.kD_PIVOT);
        // pivotController.setOutputRange(IntakeConstants.MIN_POSITION, IntakeConstants.MAX_POSITION);

        pivotMotor.getEncoder().setPositionConversionFactor(1.0 / IntakeConstants.GEAR_REDUCTION);
    
        initShuffleboard();
    }

    public void toggleState(){
        down = !down;
    }

    public boolean isDown() {
        return down;
    }

    public boolean checkCurrentSpike(){
        return pivotMotor.getOutputCurrent() > IntakeConstants.CURRENT_SPIKE_THRESHOLD;

    }

    public double getPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public double getDegrees() {
        return (pivotMotor.getEncoder().getPosition() / IntakeConstants.GEAR_REDUCTION) * 360.0;
    }

    public void pivotStop(){
        pivotMotor.set(0);
    }

    public void setGoal(double position) {
        pivotController.setGoal(position);
    }

    public void travelToSetpoint() {
        pivotMotor.set(pivotController.calculate(getPosition()));
    }

    public boolean hasReachedSetpoint() {
        return pivotController.atSetpoint();
    }

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
        intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeStop() {
        intakeMotor.set(0);
    }

    private void initShuffleboard() {
        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.INTAKE);
        ShuffleboardLayout persianPositions = moduleData.getLayout("Persian Positions", BuiltInLayouts.kList);

        persianPositions.addNumber("Intake Motor Position", () -> intakeMotor.getEncoder().getPosition());

        persianPositions.addNumber("Pivot Motor Position", () -> pivotMotor.getEncoder().getPosition());
        persianPositions.addNumber("Pivot Motor 2 Position", () -> pivotMotor.getEncoder().getPosition());
        persianPositions.addNumber("Pivot Position Degrees", this::getDegrees);

        persianPositions.addDouble("Current", () -> pivotMotor.getOutputCurrent());

        persianPositions.withSize(2, 4);

    }

}