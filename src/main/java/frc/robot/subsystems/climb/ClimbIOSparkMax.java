package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ClimbConstants;

public class ClimbIOSparkMax implements ClimbIO {
    private CANSparkMax climbMotorLeft, climbMotorRight;
    private RelativeEncoder encoderLeft, encoderRight;

    public ClimbIOSparkMax() {
        climbMotorLeft =
            new CANSparkMax(ClimbConstants.CLIMB_PORT_L, MotorType.kBrushless);
        climbMotorRight =
            new CANSparkMax(ClimbConstants.CLIMB_PORT_R, MotorType.kBrushless);
            
        climbMotorLeft.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorLeft.setSecondaryCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorLeft.setIdleMode(IdleMode.kBrake);
        climbMotorLeft.setInverted(true);

        climbMotorRight.setSmartCurrentLimit(ClimbConstants.SMART_MAX_CURRENT);
        climbMotorRight.setSecondaryCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorRight.setIdleMode(IdleMode.kBrake);

        encoderLeft = climbMotorLeft.getEncoder();
        // encoderLeft.setInverted(true);

        encoderRight = climbMotorRight.getEncoder();

        climbMotorRight.follow(climbMotorLeft, false);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.rightPos = getRightPos();
        inputs.leftPos = getLeftPos();
        inputs.highestPos = getPosition();
        inputs.isClimbAligned = isClimbAligned();
    }

    /**
     * Stops both motors
     */
    @Override
    public void stop() {
        climbMotorLeft.stopMotor();
        climbMotorRight.stopMotor();
    }

    /**
     * Resets the climb encoders
     */
    @Override
    public void reset() {
        encoderLeft.setPosition(0);
        encoderRight.setPosition(0);
    }
    
    /**
     * Returns the current position of the right Climb arm from Encoder
     */
    public double getRightPos() {
        return encoderRight.getPosition();
    }

    /**
     * Returns the current position of the left Climb arm from Encoder
     */
    public double getLeftPos() {
        return encoderLeft.getPosition();
    }

    /**
     * Returns the highest position of the climb arms from Encoder
     */
    public double getPosition() {
        return getRightPos() > getLeftPos() ? getRightPos() : getLeftPos();
    }

    /**
     * Returns whether or not the climb is properly aligned
     */
    public boolean isClimbAligned() {
        return Math.abs(getRightPos() - getLeftPos()) < ClimbConstants.MAX_VARIANCE;
    }
}
