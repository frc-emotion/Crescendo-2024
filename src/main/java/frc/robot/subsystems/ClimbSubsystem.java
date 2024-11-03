package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.TabManager;
import frc.robot.util.TabManager.SubsystemTab;

/**
 * Climb Subsystem
 */
public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax climbMotorLeft, climbMotorRight;
    private RelativeEncoder leftEncoder, rightEncoder;

    // private SparkPIDController controller;

    /**
     * Constructs a ClimbSubsystem instance
     */
    public ClimbSubsystem() {
        // Initialize Motors
        climbMotorLeft =
            new CANSparkMax(ClimbConstants.CLIMB_PORT_L, MotorType.kBrushless);
        climbMotorRight =
            new CANSparkMax(ClimbConstants.CLIMB_PORT_R, MotorType.kBrushless);

        // Set Current Limits
        climbMotorLeft.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorLeft.setSecondaryCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorLeft.setIdleMode(IdleMode.kBrake);
        climbMotorLeft.setInverted(true);

        climbMotorRight.setSmartCurrentLimit(ClimbConstants.SMART_MAX_CURRENT);
        climbMotorRight.setSecondaryCurrentLimit(ClimbConstants.CURRENT_LIMIT);
        climbMotorRight.setIdleMode(IdleMode.kBrake);

        // Get Encoders
        leftEncoder = climbMotorLeft.getEncoder();
        rightEncoder = climbMotorRight.getEncoder();

        // Get PID controllers
        // controller = climbMotorLeft.getPIDController();

        // Set Climb Motor 2 to follow Climb Motor 1
        climbMotorRight.follow(climbMotorLeft, true);

        // PID Stuff
        // controller.setOutputRange(
        // ClimbConstants.PID_MIN_OUTPUT,
        // ClimbConstants.PID_MAX_OUTPUT
        // );
        // controller.setFeedbackDevice(leftEncoder);
        // controller.setP(ClimbConstants.kP);
        // controller.setI(ClimbConstants.kI);
        // controller.setD(ClimbConstants.kD);
        // controller.setSmartMotionAccelStrategy(
        // AccelStrategy.kTrapezoidal,
        // ClimbConstants.SLOT_ID
        // );
        // controller.setSmartMotionMaxVelocity(
        // ClimbConstants.MAX_VELOCITY,
        // ClimbConstants.SLOT_ID
        // );
        // controller.setSmartMotionMaxAccel(
        // ClimbConstants.MAX_ACCELERATION,
        // ClimbConstants.SLOT_ID
        // );

        initShuffleboard();
    }

    /**
     * Sets the current position
     *
     * @param pos The position to climb to
     */
    // public void setPosition(double pos) {
    // if (pos > ClimbConstants.EXTENSION_LIMIT) {
    // pos = ClimbConstants.EXTENSION_LIMIT;
    // DriverStation.reportWarning(
    // "Climb position was set above the extension limit.",
    // false
    // );
    // }

    // controller.setReference(pos, ControlType.kSmartMotion);
    // }

    /**
     * Gets the position using the left encoder
     *
     * @return Current position from left motor encoder
     */
    // public double getPosition() {
    // return leftEncoder.getPosition();
    // }

    /**
     * Returns left motor current
     *
     * @return Output current of left motor
     */
    public double getLeftCurrent() {
        return climbMotorLeft.getOutputCurrent();
    }

    /**
     * Returns right motor current
     *
     * @return Output current of the right motor
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

    /**
     * Resets encoders
     */
    public void reset() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Initialize Climb Shuffleboard Information
     */
    private void initShuffleboard() {
        if (!Constants.DEBUG_MODE_ACTIVE) return;

        ShuffleboardTab moduleData = TabManager
            .getInstance()
            .accessTab(SubsystemTab.CLIMB);
        ShuffleboardLayout persianPositions = moduleData.getLayout(
            "Persian Positions",
            BuiltInLayouts.kList
        );

        persianPositions.addNumber(
            "Left Climb Position",
            () -> leftEncoder.getPosition()
        );

        persianPositions.addNumber(
            "Right Climb Position",
            () -> rightEncoder.getPosition()
        );

        persianPositions.addDouble(
            "Left Climb Current",
            () -> climbMotorLeft.getOutputCurrent()
        );

        persianPositions.addDouble(
            "Right Climb Current",
            () -> climbMotorRight.getOutputCurrent()
        );

        persianPositions.withSize(2, 4);
    }

    /**
     * Set speed of climb motors
     * @param speed Speed to set climb motors to 0 to 1
     */
    public void setRawSpeed(double speed) {
        climbMotorLeft.set(speed);
        climbMotorRight.set(speed);
    }

    /**
     * Get the current position of Climb from Encoder
     * @return Climb's Encoder Position (uses right encoder)
     */
    public double getPosition() {
        return rightEncoder.getPosition();
    }
}
