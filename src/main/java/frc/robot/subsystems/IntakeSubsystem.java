package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    private final SparkPIDController pivotController;

    public boolean down;
    
    public IntakeSubsystem() {

        down = false; 

        pivotMotor = new CANSparkMax(IntakeConstants.INTAKE_PIVOT_PORT, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setIdleMode(IdleMode.kCoast);
 
        pivotMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotController = pivotMotor.getPIDController();
        pivotController.setP(IntakeConstants.kP_PIVOT);
        pivotController.setI(IntakeConstants.kI_PIVOT);
        pivotController.setD(IntakeConstants.kD_PIVOT);
        pivotController.setOutputRange(IntakeConstants.MIN_POSITION, IntakeConstants.MAX_POSITION);
    
        initShuffleboard();
    }

    public void toggleState(){
        down = !down;
    }

    public boolean isDown() {
        return down == true;
    }

    public void setReference(double position) {
        pivotController.setReference(position, ControlType.kPosition);
    }

    public boolean checkCurrentSpike(){
        if(pivotMotor.getOutputCurrent() > IntakeConstants.CURRENT_SPIKE_THRESHOLD) {
            return true;
        }
        return false;
    }

    public double getPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void pivotStop(){
        pivotMotor.set(0);
    }

    public void simplePivot() {
        pivotMotor.set(IntakeConstants.INTAKE_PIVOT_SPEED);
    }

    public void stopSimple() {
        pivotMotor.set(0.0);
    }

    public void revSimplePivot() {
        pivotMotor.set(-IntakeConstants.INTAKE_PIVOT_SPEED);
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

    private void initShuffleboard() {
        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.INTAKE);
        ShuffleboardLayout persianPositions = moduleData.getLayout("Persian Positions", BuiltInLayouts.kList);

        persianPositions.addNumber("Intake Motor Position", () -> intakeMotor.get());

        persianPositions.addNumber("Pivot Motor Position", () -> pivotMotor.get());

        persianPositions.addDouble("Current", () -> pivotMotor.getOutputCurrent());

        persianPositions.withSize(2, 4);

    }

}