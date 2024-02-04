package frc.robot.subsystems;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climb Subsystem
 * 
 * @author Jason Ballinger
 * @version 2/3/2024
 */
public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax climbMotorLeft, climbMotorRight;
    private RelativeEncoder leftEncoder; // , rightEncoder;
    private SparkPIDController controller;

    /**
     * Constructs a ClimbSubsystem instance
     */
    public ClimbSubsystem() {
        // Initialize Motors
        climbMotorLeft = new CANSparkMax(ClimbConstants.CLIMB_PORT_L, MotorType.kBrushless);
        climbMotorRight = new CANSparkMax(ClimbConstants.CLIMB_PORT_R, MotorType.kBrushless);

        // Set Current Limits
        climbMotorLeft.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorLeft.setSecondaryCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorLeft.setIdleMode(IdleMode.kBrake);
        climbMotorRight.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorRight.setSecondaryCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorRight.setIdleMode(IdleMode.kBrake);

        // Get Encoders
        leftEncoder = climbMotorLeft.getEncoder();
        // rightEncoder = climbMotorRight.getEncoder();

        // Get PID controllers
        controller = climbMotorLeft.getPIDController();

        // Set Climb Motor 2 to follow Climb Motor 1
        // TODO: Test with + without, there may be a delay
        climbMotorRight.follow(climbMotorLeft);

        // PID Stuff
        controller.setOutputRange(ClimbConstants.PID_MIN_OUTPUT, ClimbConstants.PID_MAX_OUTPUT);
        controller.setFeedbackDevice(leftEncoder);
        controller.setP(ClimbConstants.kP);
        controller.setI(ClimbConstants.kI);
        controller.setD(ClimbConstants.kD);
        controller.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ClimbConstants.SLOT_ID);
        controller.setSmartMotionMaxVelocity(ClimbConstants.MAX_VELOCITY, ClimbConstants.SLOT_ID);
        controller.setSmartMotionMaxAccel(ClimbConstants.MAX_ACCELERATION, ClimbConstants.SLOT_ID);
    }

    /**
     * Sets the current position
     * 
     * @param pos The position to climb to
     */
    public void setPosition(double pos) {
        if (pos > ClimbConstants.EXTENSION_LIMIT)
            pos = ClimbConstants.EXTENSION_LIMIT;
        controller.setReference(pos, ControlType.kSmartMotion);
    }

    /**
     * Returns left motor current
     * 
     * @return the output current of left motor
     */
    public double getLeftCurrent() {
        return climbMotorLeft.getOutputCurrent();
    }

    /**
     * Returns right motor current
     * 
     * @return the output current of the right motor
     */
    public double getRightCurrent() {
        return climbMotorRight.getOutputCurrent();
    }

    /**
     * Stops both motors
     */
    public void stop() {
        climbMotorLeft.stopMotor();
        climbMotorRight.stopMotor();
    }
}
