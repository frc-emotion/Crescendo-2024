package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class PivotSubsystem extends SubsystemBase {
    Supplier<Double> yJoystick, dPad;
    Supplier<Boolean> xButton, yButton, leftStickButton;

    private CANSparkMax pivotMotor;
    private DigitalInput lowerLimit;

    public PivotSubsystem(Supplier<Double> yJoystick, Supplier<Boolean> xButton, Supplier<Boolean> yButton, Supplier<Boolean> leftStickButton) {
        this.pivotMotor = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        this.lowerLimit = new DigitalInput(Constants.LOWER_LIMIT_PORT);
        this.yJoystick = yJoystick;

        pivotMotor.getPIDController().setP(Constants.PIVOT_KP);
        pivotMotor.getPIDController().setI(Constants.PIVOT_KI);
        pivotMotor.getPIDController().setD(Constants.PIVOT_KD);
        pivotMotor.getPIDController().setFF(Constants.PIVOT_KF);

        pivotMotor.getPIDController().setOutputRange(-Constants.PIVOT_AUTO_SPEED - 0.05, Constants.PIVOT_AUTO_SPEED);

    }

    private void calibrate() {
        pivotMotor.set(0);
        pivotMotor.getEncoder().setPosition(0);
    }

     private double getRev(){
        return -pivotMotor.getEncoder().getPosition();
    }

    private void teleopRun(){
        if(!lowerLimit.get() && yJoystick.get() > 0.0){
            calibrate();
            return;
        }

        if(getRev() < Constants.PIVOT_ZERO_THRESHOLD && yJoystick.get() > 0.0){
            pivotMotor.set(yJoystick.get() * Constants.PIVOT_ZERO_SPEED);
            return;
        }

        if(getRev() > Constants.PIVOT_MAX_REVOLUTION && yJoystick.get() < 0.0){
            return;
        }
        pivotMotor.set(yJoystick.get() * Constants.PIVOT_TELEOP_SPEED);
    }

    private void stop() {
        pivotMotor.set(0);
    }

    public void run() {
        if(xButton.get()) {
            calibrate();
        } else if (Math.abs(yJoystick.get()) > Constants.TRIGGER_THRESHOLD) {
            teleopRun();
        }else if (leftStickButton.get()){
            setRev(0);
        } else {
            switch(dPad.get().intValue()){
                case 0:
                    setAgainst();
                    break;
                case 90:
                    setLine();
                    break;
                case 180:
                    setWheel();
                    break;
                case 270:
                    setTrench();
                    break;
                default:
                    stop();
                    break;
            }
        }
    }

    private void setAgainst() {
        setRev(79); // TODO: Needs to be changed
    }

    private void setLine() {
        setRev(16.35); // TODO: Needs to be changed
    }

    private void setWheel() {
        setRev(33); // TODO: Needs to be changed
    }

    private void setTrench() {
        setRev(7.5); // TODO: Needs to be changed
    }


    private void setRev(double rev) {
        double target = rev;
        if(rev<Constants.PIVOT_ZERO_THRESHOLD){
            target = 0;
        }
        if(rev>Constants.PIVOT_MAX_REVOLUTION){
            target = Constants.PIVOT_MAX_REVOLUTION;
        }
        pivotMotor.getPIDController().setReference(-target, ControlType.kPosition);
    }

    
}
