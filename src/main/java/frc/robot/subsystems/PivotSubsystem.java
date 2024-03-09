package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private SparkPIDController pivotPID;
    private RelativeEncoder relativeEncoder;
    
    private int index = 0;

    private boolean calibration = false;

    public PivotSubsystem() {

        pivotMotor = new CANSparkMax(Constants.PivotConstants.PIVOT_PORT, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(Constants.PivotConstants.MAX_CURRENT_SMART);
        pivotMotor.setSecondaryCurrentLimit(Constants.PivotConstants.MAX_CURRENT);

        pivotPID = pivotMotor.getPIDController();

        pivotPID.setP(Constants.PivotConstants.PIVOT_KP);
        pivotPID.setI(Constants.PivotConstants.PIVOT_KI);
        pivotPID.setD(Constants.PivotConstants.PIVOT_KD);
        pivotPID.setFF(Constants.PivotConstants.PIVOT_KF);

        pivotPID.setOutputRange(-Constants.PivotConstants.PIVOT_AUTO_SPEED,
                Constants.PivotConstants.PIVOT_AUTO_SPEED);

        relativeEncoder = pivotMotor.getEncoder();

        relativeEncoder.setPositionConversionFactor((-1.0 / PivotConstants.GEAR_REDUCTION) * 360);
        resetPosition(-60.0);

        initShuffleboard();
    }

    private void initShuffleboard() {
        if(!Constants.DEBUG_MODE_ACTIVE) return;

        ShuffleboardTab tab = TabManager.getInstance().accessTab(SubsystemTab.PIVOT);
        ShuffleboardLayout layout = tab.getLayout("Persian Positions", BuiltInLayouts.kList);

        layout.addDouble("Pivot Position Revolutions", this::getRev);
        layout.addDouble("Pivot Position Degrees", this::getDegrees);
        //layout.addDouble("Pivot Current", this::getCurrent);
        layout.addDouble("Pivot Preset Index", this::getIndex);
        layout.addNumber("Current Preset", this::getPreset);

        layout.withSize(2, 4);
    }

    public void resetPosition(double offset) {
        relativeEncoder.setPosition(offset);
    }

    public int getIndex() {
        return index;
    }

    public void addIndex() {
        if (Constants.PivotConstants.PIVOT_POSITIONS.length - 1 != index) {
            index++;
        }
    }

    public void subtractIndex() {
        if (index != 0) {
            index--;
        }
    }

    public void setIndex(int i) {
        index = i;
    }

    public void goToPreset() {
        setRev(Constants.PivotConstants.PIVOT_POSITIONS[index]);
    }

    public double getPreset() {
        return Constants.PivotConstants.PIVOT_POSITIONS[index];
    }

    public double getCurrent() {
        return pivotMotor.getAppliedOutput();
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public boolean getCalibration() {
        return calibration;
    }

    public void endCalibrate() {
        calibration = false;
        
        pivotMotor.set(0.0);
        
        relativeEncoder.setPosition(0);
    }

    public void calibrate() {
        calibration = true;
        pivotMotor.set(-0.2);
    }

    public double getRev() {
        return relativeEncoder.getPosition();
    }

    public double getDegrees() {
        return getRev();
    }

    public boolean isHandoffOk() {
        return this.getDegrees() - PivotConstants.kHANDOFF_ANGLE <= PivotConstants.kMAX_ANGLE_ERROR;
    }

    public void stop() {
        pivotMotor.set(0);
    }

    public void setRev(double target) {
        // double target = rev;
        // if (rev < Constants.PivotConstants.PIVOT_MIN_REVOLUTION) {
        //     target = 0;
        // }
        // if (rev > Constants.PivotConstants.PIVOT_MAX_REVOLUTION) {
        //     target = Constants.PivotConstants.PIVOT_MAX_REVOLUTION;
        // }
        pivotPID.setReference(target, ControlType.kPosition);
    }

    public void goToHandoff(){
        this.setRev(PivotConstants.kHANDOFF_ANGLE);
    }

    public boolean isAtTarget() {
        return Math.abs(relativeEncoder.getVelocity() - getPreset()) < PivotConstants.MAX_ERROR;
    }

}
